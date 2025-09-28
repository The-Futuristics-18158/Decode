package org.firstinspires.ftc.teamcode.Test;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Subsystems.*;

/**
 * Simple test class to validate the SwerveDrivePoseEstimator implementation.
 * 
 * This demonstrates basic functionality and can be used to verify that the
 * ported classes work as expected. In a real FTC environment, you would
 * replace the simulated values with actual sensor readings.
 */
public class SwervePoseEstimatorTest {
    
    /**
     * Test basic kinematics functionality
     */
    public static void testKinematics() {
        System.out.println("=== Testing SwerveDriveKinematics ===");
        
        // Create kinematics for a square robot with 0.5m wheel base
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.25, 0.25),   // Front Left
            new Translation2d(0.25, -0.25),  // Front Right
            new Translation2d(-0.25, 0.25),  // Back Left
            new Translation2d(-0.25, -0.25)  // Back Right
        );
        
        // Test forward motion
        ChassisSpeeds forwardMotion = new ChassisSpeeds(1.0, 0.0, 0.0);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(forwardMotion);
        
        System.out.println("Forward motion (1 m/s):");
        for (int i = 0; i < states.length; i++) {
            System.out.printf("  Module %d: %.2f m/s at %.2f degrees%n", 
                i, states[i].speedMetersPerSecond, Math.toDegrees(states[i].angle.getRadians()));
        }
        
        // Test rotation
        ChassisSpeeds rotation = new ChassisSpeeds(0.0, 0.0, 1.0);
        states = kinematics.toSwerveModuleStates(rotation);
        
        System.out.println("Pure rotation (1 rad/s):");
        for (int i = 0; i < states.length; i++) {
            System.out.printf("  Module %d: %.2f m/s at %.2f degrees%n", 
                i, states[i].speedMetersPerSecond, Math.toDegrees(states[i].angle.getRadians()));
        }
        
        // Test inverse kinematics
        ChassisSpeeds recovered = kinematics.toChassisSpeeds(states);
        System.out.printf("Recovered chassis speeds: vx=%.3f, vy=%.3f, omega=%.3f%n",
            recovered.vxMetersPerSecond, recovered.vyMetersPerSecond, recovered.omegaRadiansPerSecond);
        
        System.out.println();
    }
    
    /**
     * Test module state optimization
     */
    public static void testModuleOptimization() {
        System.out.println("=== Testing SwerveModuleState Optimization ===");
        
        // Test case where module needs to turn 180+ degrees
        Rotation2d currentAngle = new Rotation2d(0); // Facing forward
        SwerveModuleState desiredState = new SwerveModuleState(2.0, new Rotation2d(Math.PI)); // Facing backward
        
        System.out.printf("Before optimization: %.2f m/s at %.2f degrees%n", 
            desiredState.speedMetersPerSecond, Math.toDegrees(desiredState.angle.getRadians()));
        
        SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, currentAngle);
        
        System.out.printf("After optimization: %.2f m/s at %.2f degrees%n",
            optimized.speedMetersPerSecond, Math.toDegrees(optimized.angle.getRadians()));
        
        System.out.println();
    }
    
    /**
     * Test basic odometry functionality
     */
    public static void testOdometry() {
        System.out.println("=== Testing SwerveDriveOdometry ===");
        
        // Create kinematics
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.25, 0.25),   // Front Left
            new Translation2d(0.25, -0.25),  // Front Right
            new Translation2d(-0.25, 0.25),  // Back Left
            new Translation2d(-0.25, -0.25)  // Back Right
        );
        
        // Initial conditions
        Rotation2d initialGyro = new Rotation2d(0);
        SwerveModulePosition[] initialPositions = {
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0))
        };
        
        SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics, initialGyro, initialPositions, new Pose2d());
        
        System.out.printf("Initial pose: %s%n", odometry.getPoseMeters());
        
        // Simulate forward motion for 1 meter
        SwerveModulePosition[] newPositions = {
            new SwerveModulePosition(1.0, new Rotation2d(0)),
            new SwerveModulePosition(1.0, new Rotation2d(0)),
            new SwerveModulePosition(1.0, new Rotation2d(0)),
            new SwerveModulePosition(1.0, new Rotation2d(0))
        };
        
        Pose2d newPose = odometry.update(initialGyro, newPositions);
        System.out.printf("After 1m forward: %s%n", newPose);
        
        System.out.println();
    }
    
    /**
     * Test pose estimator with vision
     */
    public static void testPoseEstimator() {
        System.out.println("=== Testing SwerveDrivePoseEstimator ===");
        
        // Create kinematics
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.25, 0.25),   // Front Left
            new Translation2d(0.25, -0.25),  // Front Right
            new Translation2d(-0.25, 0.25),  // Back Left
            new Translation2d(-0.25, -0.25)  // Back Right
        );
        
        // Initial conditions
        Rotation2d initialGyro = new Rotation2d(0);
        SwerveModulePosition[] initialPositions = {
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0))
        };
        
        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, initialGyro, initialPositions, new Pose2d());
        
        System.out.printf("Initial pose: %s%n", poseEstimator.getEstimatedPosition());
        
        // Simulate some motion
        SwerveModulePosition[] newPositions = {
            new SwerveModulePosition(0.5, new Rotation2d(0)),
            new SwerveModulePosition(0.5, new Rotation2d(0)),
            new SwerveModulePosition(0.5, new Rotation2d(0)),
            new SwerveModulePosition(0.5, new Rotation2d(0))
        };
        
        Pose2d estimatedPose = poseEstimator.update(initialGyro, newPositions);
        System.out.printf("After odometry update: %s%n", estimatedPose);
        
        // Add a vision measurement that corrects the pose slightly
        Pose2d visionPose = new Pose2d(0.6, 0.1, new Rotation2d(Math.toRadians(5)));
        poseEstimator.addVisionMeasurement(visionPose, System.currentTimeMillis() / 1000.0);
        
        System.out.printf("After vision correction: %s%n", poseEstimator.getEstimatedPosition());
        
        System.out.println();
    }
    
    /**
     * Run all tests
     */
    public static void runAllTests() {
        System.out.println("Running SwerveDrivePoseEstimator Tests");
        System.out.println("=====================================");
        
        try {
            testKinematics();
            testModuleOptimization();
            testOdometry();
            testPoseEstimator();
            
            System.out.println("All tests completed successfully!");
            
        } catch (Exception e) {
            System.err.println("Test failed with error: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Example usage in an FTC OpMode context
     */
    public static void exampleUsage() {
        System.out.println("=== Example FTC Usage ===");
        
        // This shows how you would typically use the pose estimator in FTC
        System.out.println("""
            // In your subsystem constructor:
            SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE/2, TRACK_WIDTH/2),    // FL
                new Translation2d(WHEEL_BASE/2, -TRACK_WIDTH/2),   // FR  
                new Translation2d(-WHEEL_BASE/2, TRACK_WIDTH/2),   // BL
                new Translation2d(-WHEEL_BASE/2, -TRACK_WIDTH/2)   // BR
            );
            
            SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getGyroAngle(),
                getModulePositions(), 
                new Pose2d()
            );
            
            // In your periodic() method:
            poseEstimator.update(getGyroAngle(), getModulePositions());
            
            // When you get AprilTag data:
            if (aprilTagVisible) {
                Pose2d robotPose = calculatePoseFromAprilTag(tagData);
                poseEstimator.addVisionMeasurement(robotPose, tagData.getTimestamp());
            }
            
            // Get current position:
            Pose2d currentPose = poseEstimator.getEstimatedPosition();
            """);
    }
}