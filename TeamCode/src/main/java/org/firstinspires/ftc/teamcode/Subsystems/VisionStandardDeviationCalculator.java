package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.ejml.simple.SimpleMatrix;

/**
 * Utility class for dynamically adjusting vision measurement standard deviations
 * based on various factors like distance to targets, tag size, lighting conditions, etc.
 * 
 * This implements the concept you mentioned where camera error should increase with
 * distance from AprilTags, and allows for sophisticated error modeling.
 */
public class VisionStandardDeviationCalculator {
    
    // Default standard deviations for close, ideal measurements
    private double m_baseXStdDev = 0.1;      // meters
    private double m_baseYStdDev = 0.1;      // meters
    private double m_baseRotStdDev = 0.1;    // radians
    
    // Distance-based scaling factors
    private double m_distanceScalingFactor = 0.2;    // how much error increases with distance
    private double m_maxDistance = 6.0;              // maximum useful distance
    private double m_minDistance = 0.5;              // minimum distance before error floors out
    
    // Tag size factors (larger tags are more accurate at distance)
    private double m_tagSizeScalingFactor = 0.5;     // how much tag size affects accuracy
    
    // Angle factors (perpendicular views are more accurate)
    private double m_angleScalingFactor = 0.3;       // how much viewing angle affects accuracy
    
    // Multiple tag bonus (seeing multiple tags improves accuracy)
    private double m_multiTagBonus = 0.7;            // multiply error by this when seeing multiple tags
    
    /**
     * Calculates standard deviations based on distance to the nearest AprilTag.
     * 
     * @param distanceToTag Distance to the nearest AprilTag in meters
     * @return Standard deviations [x, y, rotation] adjusted for distance
     */
    public SimpleMatrix calculateStdDevsFromDistance(double distanceToTag) {
        return calculateStdDevsFromDistance(distanceToTag, 1.0, 0.0, 1);
    }
    
    /**
     * Calculates standard deviations based on multiple factors.
     * 
     * @param distanceToTag Distance to the nearest AprilTag in meters
     * @param tagSize Size of the AprilTag (normalized, 1.0 = standard size)
     * @param viewingAngle Angle from perpendicular to tag (0 = straight on, π/2 = edge view)
     * @param numberOfTags Number of tags visible in the measurement
     * @return Standard deviations [x, y, rotation] adjusted for all factors
     */
    public SimpleMatrix calculateStdDevsFromDistance(double distanceToTag, 
                                                   double tagSize, 
                                                   double viewingAngle, 
                                                   int numberOfTags) {
        
        // Clamp distance to reasonable bounds
        distanceToTag = Math.max(m_minDistance, Math.min(m_maxDistance, distanceToTag));
        
        // Distance factor: error increases quadratically with distance
        double distanceFactor = 1.0 + m_distanceScalingFactor * distanceToTag * distanceToTag;
        
        // Tag size factor: smaller tags are harder to detect accurately at distance
        double tagSizeFactor = 1.0 + m_tagSizeScalingFactor * (1.0 - tagSize);
        
        // Angle factor: non-perpendicular views are less accurate
        double angleFactor = 1.0 + m_angleScalingFactor * Math.abs(Math.sin(viewingAngle));
        
        // Multiple tag bonus: seeing multiple tags improves accuracy
        double multiTagFactor = numberOfTags > 1 ? m_multiTagBonus : 1.0;
        
        // Combine all factors
        double totalFactor = distanceFactor * tagSizeFactor * angleFactor * multiTagFactor;
        
        // Apply to base standard deviations
        SimpleMatrix stdDevs = new SimpleMatrix(3, 1);
        stdDevs.set(0, 0, m_baseXStdDev * totalFactor);
        stdDevs.set(1, 0, m_baseYStdDev * totalFactor);
        stdDevs.set(2, 0, m_baseRotStdDev * totalFactor);
        
        return stdDevs;
    }
    
    /**
     * Calculates distance from robot pose to the nearest AprilTag.
     * 
     * @param robotPose Current robot pose
     * @param tagPoses Array of known AprilTag poses on the field
     * @return Distance to the nearest tag in meters
     */
    public double calculateDistanceToNearestTag(Pose2d robotPose, Pose2d[] tagPoses) {
        if (tagPoses == null || tagPoses.length == 0) {
            return m_maxDistance; // No tags known, assume maximum distance
        }
        
        double minDistance = Double.MAX_VALUE;
        Translation2d robotPosition = robotPose.getTranslation();
        
        for (Pose2d tagPose : tagPoses) {
            double distance = robotPosition.getDistance(tagPose.getTranslation());
            minDistance = Math.min(minDistance, distance);
        }
        
        return minDistance;
    }
    
    /**
     * Estimates viewing angle to a specific tag.
     * 
     * @param robotPose Current robot pose
     * @param tagPose Pose of the AprilTag
     * @return Viewing angle in radians (0 = straight on, π/2 = edge view)
     */
    public double calculateViewingAngle(Pose2d robotPose, Pose2d tagPose) {
        // Vector from robot to tag
        Translation2d robotToTag = tagPose.getTranslation().minus(robotPose.getTranslation());
        
        // Tag's normal direction (facing direction)
        Translation2d tagNormal = new Translation2d(
            Math.cos(tagPose.getRotation().getRadians()),
            Math.sin(tagPose.getRotation().getRadians())
        );
        
        // Calculate angle between robot-to-tag vector and tag normal
        double dotProduct = robotToTag.getX() * (-tagNormal.getX()) + 
                           robotToTag.getY() * (-tagNormal.getY());
        double robotToTagMagnitude = robotToTag.getNorm();
        
        if (robotToTagMagnitude == 0) {
            return 0; // Robot is at tag position
        }
        
        double cosAngle = dotProduct / robotToTagMagnitude;
        cosAngle = Math.max(-1.0, Math.min(1.0, cosAngle)); // Clamp to valid range
        
        return Math.acos(Math.abs(cosAngle));
    }
    
    /**
     * Creates standard deviations that decrease trust in vision with poor conditions.
     * This is useful when you detect poor lighting, motion blur, or other issues.
     * 
     * @param confidenceLevel Confidence in the measurement (0.0 = no confidence, 1.0 = full confidence)
     * @return Standard deviations scaled by confidence level
     */
    public SimpleMatrix calculateStdDevsFromConfidence(double confidenceLevel) {
        confidenceLevel = Math.max(0.01, Math.min(1.0, confidenceLevel)); // Clamp to reasonable range
        
        // Lower confidence = higher standard deviation
        double scalingFactor = 1.0 / confidenceLevel;
        
        SimpleMatrix stdDevs = new SimpleMatrix(3, 1);
        stdDevs.set(0, 0, m_baseXStdDev * scalingFactor);
        stdDevs.set(1, 0, m_baseYStdDev * scalingFactor);
        stdDevs.set(2, 0, m_baseRotStdDev * scalingFactor);
        
        return stdDevs;
    }
    
    /**
     * Adaptive standard deviations that account for cumulative odometry error.
     * 
     * As you mentioned, odometry error grows over time/distance. This method allows
     * you to adjust vision trust based on how long it's been since the last vision correction.
     * 
     * @param timeSinceLastVision Time since last vision measurement (seconds)
     * @param distanceTraveled Distance traveled since last vision measurement (meters)
     * @return Standard deviations that give more weight to vision when odometry is suspect
     */
    public SimpleMatrix calculateStdDevsFromOdometryDrift(double timeSinceLastVision, 
                                                        double distanceTraveled) {
        // Odometry gets less trustworthy over time and distance
        double driftFactor = 1.0 + 0.1 * timeSinceLastVision + 0.05 * distanceTraveled;
        
        // As odometry becomes less trustworthy, we should trust vision more (lower std dev)
        double visionTrustFactor = 1.0 / Math.sqrt(driftFactor);
        
        SimpleMatrix stdDevs = new SimpleMatrix(3, 1);
        stdDevs.set(0, 0, m_baseXStdDev * visionTrustFactor);
        stdDevs.set(1, 0, m_baseYStdDev * visionTrustFactor);
        stdDevs.set(2, 0, m_baseRotStdDev * visionTrustFactor);
        
        return stdDevs;
    }
    
    // Configuration methods
    
    public void setBaseStandardDeviations(double x, double y, double rotation) {
        m_baseXStdDev = x;
        m_baseYStdDev = y;
        m_baseRotStdDev = rotation;
    }
    
    public void setDistanceScaling(double scalingFactor, double maxDistance, double minDistance) {
        m_distanceScalingFactor = scalingFactor;
        m_maxDistance = maxDistance;
        m_minDistance = minDistance;
    }
    
    public void setTagSizeScaling(double scalingFactor) {
        m_tagSizeScalingFactor = scalingFactor;
    }
    
    public void setAngleScaling(double scalingFactor) {
        m_angleScalingFactor = scalingFactor;
    }
    
    public void setMultiTagBonus(double bonus) {
        m_multiTagBonus = bonus;
    }
}