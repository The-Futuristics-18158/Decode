package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.ejml.simple.SimpleMatrix;

import java.util.Collections;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * This class wraps {@link SwerveDriveOdometry Swerve Drive Odometry} to fuse latency-compensated
 * vision measurements with swerve drive encoder distance measurements. It is intended to be a
 * drop-in replacement for {@link SwerveDriveOdometry}.
 *
 * <p>{@link SwerveDrivePoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link SwerveDrivePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave as regular encoder odometry.
 * 
 * Ported from WPILib's SwerveDrivePoseEstimator for FTC use.
 */
public class SwerveDrivePoseEstimator {
    private final SwerveDriveOdometry m_odometry;
    private final SimpleMatrix m_q = new SimpleMatrix(3, 1);
    private final SimpleMatrix m_visionK = new SimpleMatrix(3, 3);

    private static final double kBufferDuration = 1.5; // seconds
    private final NavigableMap<Double, Map<Integer, SwerveModulePosition>> m_poseBuffer =
        new TreeMap<>();

    /**
     * Constructs a SwerveDrivePoseEstimator with default standard deviations for the model and
     * vision measurements.
     *
     * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for
     * y, and 0.1 radians for heading. The default standard deviations of the vision measurements
     * are 0.45 meters for x, 0.45 meters for y, and 0.45 radians for heading.
     *
     * @param kinematics A correctly-configured kinematics object for your drivetrain.
     * @param gyroAngle The current gyro angle.
     * @param modulePositions The current distance measurements and rotations of the swerve modules.
     * @param initialPoseMeters The starting pose estimate.
     */
    public SwerveDrivePoseEstimator(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters) {
        this(
            kinematics,
            gyroAngle,
            modulePositions,
            initialPoseMeters,
            createMatrix(VecBuilder.fill(0.1, 0.1, 0.1)),
            createMatrix(VecBuilder.fill(0.45, 0.45, 0.45)));
    }

    /**
     * Constructs a SwerveDrivePoseEstimator.
     *
     * @param kinematics A correctly-configured kinematics object for your drivetrain.
     * @param gyroAngle The current gyro angle.
     * @param modulePositions The current distance and rotation measurements of the swerve modules.
     * @param initialPoseMeters The starting pose estimate.
     * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y
     *     position in meters, and heading in radians). Increase these numbers to trust your state
     *     estimate less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x
     *     position in meters, y position in meters, and heading in radians). Increase these
     *     numbers to trust the vision pose measurement less.
     */
    public SwerveDrivePoseEstimator(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters,
        SimpleMatrix stateStdDevs,
        SimpleMatrix visionMeasurementStdDevs) {
        
        m_odometry = new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters);

        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }

        // Initialize vision Kalman filter
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    /**
     * Sets the pose estimator's trust in vision measurements. This might be used to change trust in
     * vision measurements after the autonomous period, or to change trust as distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     *     theta]ᵀ, with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(SimpleMatrix visionMeasurementStdDevs) {
        SimpleMatrix r = new SimpleMatrix(3, 3);
        for (int i = 0; i < 3; ++i) {
            r.set(i, i, visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0));
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0.
        // See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                    row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r.get(row, row))));
            }
        }
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * <p>Similarly, module positions do not need to be reset by the user before calling this
     * method. This method will automatically reset the module positions internally.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param modulePositions The current distance measurements and rotations of the swerve modules.
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPosition(
        Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
        
        // Reset state estimate and error covariance
        m_odometry.resetPosition(gyroAngle, modulePositions, poseMeters);
        m_poseBuffer.clear();
    }

    /**
     * Gets the estimated robot pose.
     *
     * @return The estimated robot pose in meters.
     */
    public Pose2d getEstimatedPosition() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Adds a vision measurement to the Kalman filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimator and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
     *     don't use your own time source by calling {@link
     *     SwerveDrivePoseEstimator#updateWithTime(double, Rotation2d, SwerveModulePosition[])
     *     updateWithTime}, then you must use a timestamp with an epoch since FPGA startup (i.e.,
     *     the epoch of this timestamp is the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}).
     *     This means that you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as
     *     your time source or sync the epochs.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // If no samples in buffer, we can't interpolate and should just trust the odometry
        if (m_poseBuffer.isEmpty()) {
            return;
        }

        // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (m_poseBuffer.firstKey() > timestampSeconds) {
                return;
            }
        } catch (Exception ex) {
            return;
        }

        // Step 1: Get the pose odometry measured at the moment the vision measurement was made.
        Map<Integer, SwerveModulePosition> sample = m_poseBuffer.get(timestampSeconds);

        if (sample == null) {
            // Step 1.1: If there was no odometry sample at the exact timestamp we're looking
            // for, then interpolate between samples to estimate the odometry at that time.
            sample = interpolate(timestampSeconds);
            if (sample == null) {
                return;
            }
        }

        // Step 2: Measure the twist between the odometry pose and the vision pose.
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[sample.size()];
        for (int i = 0; i < sample.size(); i++) {
            modulePositions[i] = sample.get(i);
        }

        // Get odometry-only robot pose
        Pose2d odometryPose = m_odometry.getPoseMeters();

        // Step 3: We should not trust the vision measurement if the norm of the difference between
        // the vision measurement and the odometry is too high.
        Translation2d diff = visionRobotPoseMeters.getTranslation().minus(odometryPose.getTranslation());
        if (diff.getNorm() > 4) { // 4 meters
            return;
        }

        // Step 4: Apply the vision measurement as a correction to the current pose estimate
        SimpleMatrix visionMeasurement = poseToVector(visionRobotPoseMeters);
        SimpleMatrix odometryMeasurement = poseToVector(odometryPose);
        SimpleMatrix residual = visionMeasurement.minus(odometryMeasurement);

        // Apply the correction using Kalman filter gain
        SimpleMatrix correction = m_visionK.mult(residual);
        Pose2d correctedPose = vectorToPose(odometryMeasurement.plus(correction));

        // Update the odometry with the corrected pose
        m_odometry.resetPosition(
            correctedPose.getRotation(), 
            modulePositions, 
            correctedPose);
    }

    /**
     * Adds a vision measurement to the Kalman filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimator and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * <p>Note that the vision measurement standard deviations passed into this method will only be
     * used for this measurement, and will not affect the standard deviations passed in during
     * construction.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x
     *     position in meters, y position in meters, and heading in radians). Increase these
     *     numbers to trust the vision pose measurement less.
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        SimpleMatrix visionMeasurementStdDevs) {
        
        // Save current vision measurement standard deviations
        SimpleMatrix originalVisionK = new SimpleMatrix(m_visionK);
        
        // Temporarily update vision measurement standard deviations
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        
        // Apply the vision measurement
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
        
        // Restore original vision measurement standard deviations
        m_visionK.set(originalVisionK);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called
     * every loop.
     *
     * @param gyroAngle The current gyro angle.
     * @param modulePositions The current distance measurements and rotations of the swerve modules.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        return updateWithTime(System.currentTimeMillis() / 1000.0, gyroAngle, modulePositions);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called
     * every loop.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param gyroAngle The current gyro angle.
     * @param modulePositions The current distance measurements and rotations of the swerve modules.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d updateWithTime(
        double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        
        Pose2d odometryEstimate = m_odometry.update(gyroAngle, modulePositions);

        m_poseBuffer.put(
            currentTimeSeconds,
            cloneModulePositions(modulePositions));

        // Remove old entries
        cleanUpBuffer(currentTimeSeconds);

        return odometryEstimate;
    }

    /**
     * Removes stale pose estimates that are older than our buffer duration.
     */
    private void cleanUpBuffer(double currentTimeSeconds) {
        double oldestTime = currentTimeSeconds - kBufferDuration;
        while (!m_poseBuffer.isEmpty() && m_poseBuffer.firstKey() < oldestTime) {
            m_poseBuffer.pollFirstEntry();
        }
    }

    /**
     * Clones a swerve module position array.
     */
    private Map<Integer, SwerveModulePosition> cloneModulePositions(SwerveModulePosition[] modulePositions) {
        Map<Integer, SwerveModulePosition> map = new TreeMap<>();
        for (int i = 0; i < modulePositions.length; i++) {
            map.put(i, new SwerveModulePosition(
                modulePositions[i].distanceMeters, 
                modulePositions[i].angle));
        }
        return map;
    }

    /**
     * Interpolates between buffer samples to estimate the robot pose at the given timestamp.
     */
    private Map<Integer, SwerveModulePosition> interpolate(double timeSeconds) {
        // Get surrounding samples for interpolation
        Map.Entry<Double, Map<Integer, SwerveModulePosition>> topBound = m_poseBuffer.ceilingEntry(timeSeconds);
        Map.Entry<Double, Map<Integer, SwerveModulePosition>> bottomBound = m_poseBuffer.floorEntry(timeSeconds);

        // If no bounds, cannot interpolate
        if (topBound == null || bottomBound == null) {
            return null;
        }

        // If both bounds are the same, just return that sample
        if (topBound.equals(bottomBound)) {
            return bottomBound.getValue();
        }

        // Interpolate between the two samples
        double topBoundTime = topBound.getKey();
        double bottomBoundTime = bottomBound.getKey();
        double t = (timeSeconds - bottomBoundTime) / (topBoundTime - bottomBoundTime);

        Map<Integer, SwerveModulePosition> result = new TreeMap<>();
        Map<Integer, SwerveModulePosition> bottomBoundSample = bottomBound.getValue();
        Map<Integer, SwerveModulePosition> topBoundSample = topBound.getValue();

        for (int i = 0; i < bottomBoundSample.size(); i++) {
            SwerveModulePosition bottom = bottomBoundSample.get(i);
            SwerveModulePosition top = topBoundSample.get(i);

            double interpolatedDistance = bottom.distanceMeters + t * (top.distanceMeters - bottom.distanceMeters);
            Rotation2d interpolatedAngle = interpolateRotation2d(bottom.angle, top.angle, t);

            result.put(i, new SwerveModulePosition(interpolatedDistance, interpolatedAngle));
        }

        return result;
    }

    private SimpleMatrix poseToVector(Pose2d pose) {
        SimpleMatrix vector = new SimpleMatrix(3, 1);
        vector.set(0, 0, pose.getX());
        vector.set(1, 0, pose.getY());
        vector.set(2, 0, pose.getRotation().getRadians());
        return vector;
    }

    private Pose2d vectorToPose(SimpleMatrix vector) {
        return new Pose2d(
            vector.get(0, 0),
            vector.get(1, 0),
            new Rotation2d(vector.get(2, 0)));
    }

    public static SimpleMatrix createMatrix(double[] values) {
        SimpleMatrix matrix = new SimpleMatrix(values.length, 1);
        for (int i = 0; i < values.length; i++) {
            matrix.set(i, 0, values[i]);
        }
        return matrix;
    }

    /**
     * Interpolates between two Rotation2d objects.
     * This is a workaround since FTCLib doesn't provide the interpolate method.
     * 
     * @param start The starting rotation
     * @param end The ending rotation
     * @param t The interpolation parameter (0.0 to 1.0)
     * @return The interpolated rotation
     */
    private static Rotation2d interpolateRotation2d(Rotation2d start, Rotation2d end, double t) {
        // Find the shortest angular distance
        double startRadians = start.getRadians();
        double endRadians = end.getRadians();
        
        // Normalize the difference to [-π, π]
        double difference = endRadians - startRadians;
        while (difference > Math.PI) difference -= 2 * Math.PI;
        while (difference < -Math.PI) difference += 2 * Math.PI;
        
        // Interpolate
        double interpolatedRadians = startRadians + t * difference;
        return new Rotation2d(interpolatedRadians);
    }

    /**
     * Utility class for creating vectors.
     */
    public static class VecBuilder {
        public static double[] fill(double... values) {
            return values;
        }
    }
}