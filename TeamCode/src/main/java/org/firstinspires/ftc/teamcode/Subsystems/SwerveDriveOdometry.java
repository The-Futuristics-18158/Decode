package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's position on the field
 * over a course of a match using readings from your swerve drive encoders and swerve azimuth
 * encoders.
 *
 * <p>Teams can use odometry during the autonomous period for complex path following. Furthermore,
 * odometry can be used for latency compensation when using computer-vision systems.
 * 
 * Ported from WPILib's SwerveDriveOdometry for FTC use.
 */
public class SwerveDriveOdometry {
    private final SwerveDriveKinematics m_kinematics;
    private Pose2d m_poseMeters;

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;
    private SwerveModulePosition[] m_previousModulePositions;

    /**
     * Constructs a SwerveDriveOdometry object.
     *
     * @param kinematics The swerve drive kinematics for your drivetrain.
     * @param gyroAngle The angle reported by the gyroscope.
     * @param modulePositions The wheel positions reported by each module.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public SwerveDriveOdometry(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters) {
        
        m_kinematics = kinematics;
        m_poseMeters = initialPoseMeters;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();
        
        m_previousModulePositions = new SwerveModulePosition[modulePositions.length];
        for (int index = 0; index < modulePositions.length; index++) {
            m_previousModulePositions[index] = new SwerveModulePosition(
                modulePositions[index].distanceMeters, modulePositions[index].angle);
        }
    }

    /**
     * Constructs a SwerveDriveOdometry object with the default pose at the origin.
     *
     * @param kinematics The swerve drive kinematics for your drivetrain.
     * @param gyroAngle The angle reported by the gyroscope.
     * @param modulePositions The wheel positions reported by each module.
     */
    public SwerveDriveOdometry(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions) {
        this(kinematics, gyroAngle, modulePositions, new Pose2d());
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * <p>Similarly, module positions do not need to be reset by the user before calling this
     * method. This method will automatically reset the module positions internally.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param modulePositions The wheel positions reported by each module.
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPosition(
        Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
        
        if (modulePositions.length != m_previousModulePositions.length) {
            throw new IllegalArgumentException(
                "Number of modules is not consistent with number of wheel locations provided in "
                + "constructor");
        }

        m_poseMeters = poseMeters;
        m_previousAngle = poseMeters.getRotation();
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        
        for (int index = 0; index < modulePositions.length; index++) {
            m_previousModulePositions[index] = new SwerveModulePosition(
                modulePositions[index].distanceMeters, modulePositions[index].angle);
        }
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public Pose2d getPoseMeters() {
        return m_poseMeters;
    }

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the
     * pose over time. This method automatically calculates the current time to calculate period
     * (difference between two timestamps). The period is used to calculate the change in distance
     * over a period of time.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param modulePositions The current position of all swerve modules. Please provide the
     *     positions in the same order in which you instantiated your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        if (modulePositions.length != m_previousModulePositions.length) {
            throw new IllegalArgumentException(
                "Number of modules is not consistent with number of wheel locations provided in "
                + "constructor");
        }

        Rotation2d angle = gyroAngle.plus(m_gyroOffset);

        SwerveModuleState[] moduleStates = new SwerveModuleState[modulePositions.length];
        for (int index = 0; index < modulePositions.length; index++) {
            SwerveModulePosition currentPosition = modulePositions[index];
            SwerveModulePosition previousPosition = m_previousModulePositions[index];

            double deltaDistanceMeters = currentPosition.distanceMeters - previousPosition.distanceMeters;

            // Use the current angle for the module state
            moduleStates[index] = new SwerveModuleState(deltaDistanceMeters, currentPosition.angle);
            
            // Update the previous position
            previousPosition.distanceMeters = currentPosition.distanceMeters;
            previousPosition.angle = currentPosition.angle;
        }

        ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);

        var twist = new Twist2d(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            angle.getRadians() - m_previousAngle.getRadians());

        var newPose = m_poseMeters.exp(twist);

        m_previousAngle = angle;
        m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

        return m_poseMeters;
    }

    /**
     * Updates the robot pose using only wheel encoder information. Note that this will make the
     * pose drift over time. If you have access to a gyroscope, use the overload that includes the
     * gyro angle.
     *
     * @param modulePositions The current position of all swerve modules. Please provide the
     *     positions in the same order in which you instantiated your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose2d updateWithoutGyro(SwerveModulePosition[] modulePositions) {
        if (modulePositions.length != m_previousModulePositions.length) {
            throw new IllegalArgumentException(
                "Number of modules is not consistent with number of wheel locations provided in "
                + "constructor");
        }

        SwerveModuleState[] moduleStates = new SwerveModuleState[modulePositions.length];
        for (int index = 0; index < modulePositions.length; index++) {
            SwerveModulePosition currentPosition = modulePositions[index];
            SwerveModulePosition previousPosition = m_previousModulePositions[index];

            double deltaDistanceMeters = currentPosition.distanceMeters - previousPosition.distanceMeters;

            moduleStates[index] = new SwerveModuleState(deltaDistanceMeters, currentPosition.angle);
            
            previousPosition.distanceMeters = currentPosition.distanceMeters;
            previousPosition.angle = currentPosition.angle;
        }

        ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);

        var twist = new Twist2d(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond);

        var newPose = m_poseMeters.exp(twist);
        m_poseMeters = newPose;

        return m_poseMeters;
    }
}