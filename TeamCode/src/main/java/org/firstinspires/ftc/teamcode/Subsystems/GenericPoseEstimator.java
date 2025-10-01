package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.ejml.simple.SimpleMatrix;

import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.Map;

/**
 * A generic pose estimator that fuses direct odometry pose estimates with vision measurements.
 * 
 * Unlike drive-specific estimators that work with individual wheel encoder counts, this estimator
 * accepts direct Pose2d measurements from any odometry system (odometry pods, Pinpoint sensor,
 * Rev odometry computer, etc.) and supplements them with vision corrections using Kalman filtering.
 * 
 * Key advantages:
 * - Works with any drive type (swerve, mecanum, differential, holonomic)
 * - Separates concerns: odometry system handles pose calculation, estimator handles fusion
 * - Maintains historical pose buffer for latency compensation
 * - Uses statistical filtering to optimally weight odometry vs vision measurements
 * 
 * Usage pattern:
 * 1. Call update() every robot loop with the latest odometry pose
 * 2. Call addVisionMeasurement() whenever vision data is available
 * 3. Call getEstimatedPose() to get the current best estimate
 */
public class GenericPoseEstimator {
    
    // Kalman filter matrices
    private final SimpleMatrix m_q = new SimpleMatrix(3, 1);  // Process noise (odometry uncertainty)
    private final SimpleMatrix m_visionK = new SimpleMatrix(3, 3);  // Kalman gain matrix
    
    // Current state
    private Pose2d m_currentPose;
    private double m_lastUpdateTime;
    
    // Historical pose buffer for latency compensation
    private static final double kBufferDuration = 2.0; // seconds
    private final NavigableMap<Double, Pose2d> m_poseBuffer = new TreeMap<>();
    
    // Configuration
    private double m_maxVisionDistance = 4.0; // meters - reject vision measurements beyond this distance
    private boolean m_enableLatencyCompensation = true;
    
    /**
     * Constructs a GenericPoseEstimator with default standard deviations.
     * 
     * Default odometry std devs: 0.1m x, 0.1m y, 0.1 rad heading
     * Default vision std devs: 0.45m x, 0.45m y, 0.45 rad heading
     * 
     * @param initialPose The starting pose estimate
     */
    public GenericPoseEstimator(Pose2d initialPose) {
        this(initialPose, 
             createVector(0.1, 0.1, 0.1),      // Odometry standard deviations
             createVector(0.45, 0.45, 0.45));   // Vision standard deviations
    }
    
    /**
     * Constructs a GenericPoseEstimator with custom standard deviations.
     * 
     * @param initialPose The starting pose estimate
     * @param odometryStdDevs Standard deviations of odometry pose estimates [x, y, theta]
     *                        Increase to trust odometry less
     * @param visionStdDevs Standard deviations of vision pose measurements [x, y, theta]
     *                      Increase to trust vision less
     */
    public GenericPoseEstimator(Pose2d initialPose, 
                               SimpleMatrix odometryStdDevs, 
                               SimpleMatrix visionStdDevs) {
        m_currentPose = initialPose;
        m_lastUpdateTime = getCurrentTimeSeconds();
        
        // Initialize process noise matrix (odometry uncertainty)
        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, odometryStdDevs.get(i, 0) * odometryStdDevs.get(i, 0));
        }
        
        // Initialize vision measurement standard deviations
        setVisionMeasurementStdDevs(visionStdDevs);
        
        // Add initial pose to buffer
        m_poseBuffer.put(m_lastUpdateTime, m_currentPose);
    }
    
    /**
     * Updates the pose estimator with the latest odometry measurement.
     * This should be called every robot loop.
     * 
     * @param odometryPose The current pose as measured by the odometry system
     * @return The current estimated pose (same as getEstimatedPose())
     */
    public Pose2d update(Pose2d odometryPose) {
        return updateWithTime(getCurrentTimeSeconds(), odometryPose);
    }
    
    /**
     * Updates the pose estimator with the latest odometry measurement at a specific time.
     * 
     * @param timeSeconds The timestamp of this measurement
     * @param odometryPose The pose as measured by the odometry system at this time
     * @return The current estimated pose
     */
    public Pose2d updateWithTime(double timeSeconds, Pose2d odometryPose) {
        // Update our current pose estimate
        m_currentPose = odometryPose;
        m_lastUpdateTime = timeSeconds;
        
        // Add to historical buffer
        m_poseBuffer.put(timeSeconds, odometryPose);
        
        // Clean up old entries
        cleanUpBuffer(timeSeconds);
        
        return m_currentPose;
    }
    
    /**
     * Adds a vision measurement to correct the pose estimate.
     * 
     * This method implements the core fusion algorithm:
     * 1. Look up the historical odometry pose at the vision timestamp
     * 2. Calculate the correction needed based on vision vs odometry
     * 3. Apply correction using Kalman filtering
     * 4. Update current pose estimate
     * 
     * @param visionPose The robot pose as measured by vision
     * @param timestampSeconds The timestamp when the vision measurement was taken
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        addVisionMeasurement(visionPose, timestampSeconds, null);
    }
    
    /**
     * Adds a vision measurement with custom standard deviations for this measurement.
     * 
     * @param visionPose The robot pose as measured by vision
     * @param timestampSeconds The timestamp when the vision measurement was taken
     * @param visionStdDevs Standard deviations for this specific measurement [x, y, theta]
     *                      If null, uses the default standard deviations
     */
    public void addVisionMeasurement(Pose2d visionPose, 
                                   double timestampSeconds, 
                                   SimpleMatrix visionStdDevs) {
        
        // Step 0: Basic validation
        if (m_poseBuffer.isEmpty()) {
            return; // No odometry data to work with
        }
        
        // Check if measurement is too old
        if (m_poseBuffer.firstKey() > timestampSeconds) {
            return; // Outside our buffer window
        }
        
        // Step 1: Get the historical odometry pose at the vision timestamp
        Pose2d historicalOdometryPose = getHistoricalPose(timestampSeconds);
        if (historicalOdometryPose == null) {
            return; // Could not determine historical pose
        }
        
        // Step 2: Validate the vision measurement isn't too far from odometry
        Translation2d diff = visionPose.getTranslation().minus(historicalOdometryPose.getTranslation());
        if (diff.getNorm() > m_maxVisionDistance) {
            // Vision measurement is too far from odometry - likely incorrect
            return;
        }
        
        // Step 3: Calculate the correction
        SimpleMatrix visionVector = poseToVector(visionPose);
        SimpleMatrix odometryVector = poseToVector(historicalOdometryPose);
        SimpleMatrix residual = visionVector.minus(odometryVector);
        
        // Step 4: Apply Kalman filter correction
        SimpleMatrix originalVisionK = null;
        if (visionStdDevs != null) {
            // Temporarily use custom standard deviations
            originalVisionK = new SimpleMatrix(m_visionK);
            setVisionMeasurementStdDevs(visionStdDevs);
        }
        
        SimpleMatrix correction = m_visionK.mult(residual);
        
        // Step 5: Apply correction to current pose
        SimpleMatrix currentPoseVector = poseToVector(m_currentPose);
        SimpleMatrix correctedVector = currentPoseVector.plus(correction);
        m_currentPose = vectorToPose(correctedVector);
        
        // Update the buffer with corrected pose at current time
        m_poseBuffer.put(m_lastUpdateTime, m_currentPose);
        
        // Restore original vision K if we changed it
        if (originalVisionK != null) {
            m_visionK.set(originalVisionK);
        }
    }
    
    /**
     * Gets the current best estimate of the robot's pose.
     * 
     * @return The estimated robot pose
     */
    public Pose2d getEstimatedPose() {
        return m_currentPose;
    }
    
    /**
     * Resets the estimator to a known pose.
     * 
     * @param pose The known pose to reset to
     */
    public void resetPose(Pose2d pose) {
        resetPose(pose, getCurrentTimeSeconds());
    }
    
    /**
     * Resets the estimator to a known pose at a specific time.
     * 
     * @param pose The known pose to reset to
     * @param timeSeconds The timestamp of the reset
     */
    public void resetPose(Pose2d pose, double timeSeconds) {
        m_currentPose = pose;
        m_lastUpdateTime = timeSeconds;
        m_poseBuffer.clear();
        m_poseBuffer.put(timeSeconds, pose);
    }
    
    /**
     * Sets the trust level for vision measurements.
     * 
     * @param visionMeasurementStdDevs Standard deviations [x, y, theta] in meters and radians
     *                                 Larger values = less trust in vision
     */
    public void setVisionMeasurementStdDevs(SimpleMatrix visionMeasurementStdDevs) {
        SimpleMatrix r = new SimpleMatrix(3, 3);
        for (int i = 0; i < 3; ++i) {
            r.set(i, i, visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0));
        }
        
        // Calculate Kalman gain: K = Q / (Q + sqrt(Q * R))
        // This is the closed-form solution for continuous Kalman filter with A = 0
        for (int row = 0; row < 3; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(row, row, 
                    m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r.get(row, row))));
            }
        }
    }
    
    /**
     * Sets the maximum distance between vision and odometry measurements before rejection.
     * 
     * @param maxDistance Maximum allowed distance in meters
     */
    public void setMaxVisionDistance(double maxDistance) {
        m_maxVisionDistance = maxDistance;
    }
    
    /**
     * Enables or disables latency compensation.
     * 
     * @param enable True to enable latency compensation (default), false to disable
     */
    public void setLatencyCompensationEnabled(boolean enable) {
        m_enableLatencyCompensation = enable;
    }
    
    /**
     * Gets a historical pose from the buffer, with interpolation if needed.
     * 
     * @param timestamp The timestamp to look up
     * @return The pose at that timestamp, or null if not available
     */
    private Pose2d getHistoricalPose(double timestamp) {
        if (!m_enableLatencyCompensation) {
            return m_currentPose; // Just use current pose
        }
        
        // Check if we have an exact match
        Pose2d exactMatch = m_poseBuffer.get(timestamp);
        if (exactMatch != null) {
            return exactMatch;
        }
        
        // Find surrounding entries for interpolation
        Map.Entry<Double, Pose2d> lowerEntry = m_poseBuffer.floorEntry(timestamp);
        Map.Entry<Double, Pose2d> higherEntry = m_poseBuffer.ceilingEntry(timestamp);
        
        // If we can't interpolate, return the closest available
        if (lowerEntry == null && higherEntry == null) {
            return null; // No data
        } else if (lowerEntry == null) {
            return higherEntry.getValue(); // Use higher entry
        } else if (higherEntry == null) {
            return lowerEntry.getValue(); // Use lower entry
        }
        
        // Interpolate between the two entries
        double lowerTime = lowerEntry.getKey();
        double higherTime = higherEntry.getKey();
        double t = (timestamp - lowerTime) / (higherTime - lowerTime);
        
        return interpolatePose(lowerEntry.getValue(), higherEntry.getValue(), t);
    }
    
    /**
     * Interpolates between two poses.
     * 
     * @param startPose The starting pose
     * @param endPose The ending pose
     * @param t The interpolation factor (0.0 to 1.0)
     * @return The interpolated pose
     */
    private Pose2d interpolatePose(Pose2d startPose, Pose2d endPose, double t) {
        // Interpolate translation
        Translation2d startTrans = startPose.getTranslation();
        Translation2d endTrans = endPose.getTranslation();
        Translation2d interpTrans = new Translation2d(
            startTrans.getX() + t * (endTrans.getX() - startTrans.getX()),
            startTrans.getY() + t * (endTrans.getY() - startTrans.getY())
        );
        
        // Interpolate rotation (shortest path)
        Rotation2d interpRotation = interpolateRotation(startPose.getRotation(), endPose.getRotation(), t);
        
        return new Pose2d(interpTrans, interpRotation);
    }
    
    /**
     * Interpolates between two rotations using the shortest angular path.
     */
    private Rotation2d interpolateRotation(Rotation2d start, Rotation2d end, double t) {
        double startRadians = start.getRadians();
        double endRadians = end.getRadians();
        
        // Find the shortest angular distance
        double difference = endRadians - startRadians;
        while (difference > Math.PI) difference -= 2 * Math.PI;
        while (difference < -Math.PI) difference += 2 * Math.PI;
        
        // Interpolate
        double interpolatedRadians = startRadians + t * difference;
        return new Rotation2d(interpolatedRadians);
    }
    
    /**
     * Removes old entries from the pose buffer.
     */
    private void cleanUpBuffer(double currentTime) {
        double oldestAllowedTime = currentTime - kBufferDuration;
        while (!m_poseBuffer.isEmpty() && m_poseBuffer.firstKey() < oldestAllowedTime) {
            m_poseBuffer.pollFirstEntry();
        }
    }
    
    /**
     * Converts a Pose2d to a state vector for Kalman filter calculations.
     */
    private SimpleMatrix poseToVector(Pose2d pose) {
        SimpleMatrix vector = new SimpleMatrix(3, 1);
        vector.set(0, 0, pose.getX());
        vector.set(1, 0, pose.getY());
        vector.set(2, 0, pose.getRotation().getRadians());
        return vector;
    }
    
    /**
     * Converts a state vector back to a Pose2d.
     */
    private Pose2d vectorToPose(SimpleMatrix vector) {
        return new Pose2d(
            vector.get(0, 0),
            vector.get(1, 0),
            new Rotation2d(vector.get(2, 0))
        );
    }
    
    /**
     * Creates a SimpleMatrix vector from individual values.
     */
    public static SimpleMatrix createVector(double x, double y, double theta) {
        SimpleMatrix matrix = new SimpleMatrix(3, 1);
        matrix.set(0, 0, x);
        matrix.set(1, 0, y);
        matrix.set(2, 0, theta);
        return matrix;
    }
    
    /**
     * Gets the current time in seconds.
     * Uses system time - in a real robot you might want to use a more precise timer.
     */
    private double getCurrentTimeSeconds() {
        return System.nanoTime() / 1e9;
    }
}