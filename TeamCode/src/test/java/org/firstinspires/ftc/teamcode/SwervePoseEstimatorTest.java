package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.junit.Test;
import org.junit.Before;
import static org.junit.Assert.*;

/**
 * JUnit test class to validate the SwerveDrivePoseEstimator implementation.
 * 
 * Run with: ./gradlew :TeamCode:test
 */
public class SwervePoseEstimatorTest {

    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] initialPositions;
    private Rotation2d initialGyro;

    @Before
    public void setUp() {
        // Create kinematics for a square robot with 0.5m wheel base
        kinematics = new SwerveDriveKinematics(
            new Translation2d(0.25, 0.25),   // Front Left
            new Translation2d(0.25, -0.25),  // Front Right
            new Translation2d(-0.25, 0.25),  // Back Left
            new Translation2d(-0.25, -0.25)  // Back Right
        );

        // Initial conditions
        initialGyro = new Rotation2d(0);
        initialPositions = new SwerveModulePosition[]{
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0))
        };
    }

    /**
     * Test basic kinematics functionality
     */
    @Test
    public void testKinematics() {
        System.out.println("=== Testing SwerveDriveKinematics ===");
        
        // Test forward motion
        ChassisSpeeds forwardMotion = new ChassisSpeeds(1.0, 0.0, 0.0);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(forwardMotion);
        
        // All modules should have same speed for pure forward motion
        for (SwerveModuleState state : states) {
            assertEquals("Forward motion should result in 1 m/s", 1.0, state.speedMetersPerSecond, 0.01);
            assertEquals("Forward motion should be at 0 degrees", 0.0, state.angle.getRadians(), 0.01);
        }
        
        // Test rotation
        ChassisSpeeds rotation = new ChassisSpeeds(0.0, 0.0, 1.0);
        states = kinematics.toSwerveModuleStates(rotation);
        
        // Test inverse kinematics
        ChassisSpeeds recovered = kinematics.toChassisSpeeds(states);
        
        // Verify we can recover the original rotation command
        assertEquals("Should recover rotation speed", 1.0, recovered.omegaRadiansPerSecond, 0.01);
        assertEquals("Should have no forward velocity", 0.0, recovered.vxMetersPerSecond, 0.01);
        assertEquals("Should have no sideways velocity", 0.0, recovered.vyMetersPerSecond, 0.01);
        
        System.out.println("Kinematics test passed!");
    }

    /**
     * Test module state optimization
     */
    @Test
    public void testModuleOptimization() {
        System.out.println("=== Testing SwerveModuleState Optimization ===");
        
        // Test case where module needs to turn 180+ degrees
        Rotation2d currentAngle = new Rotation2d(0); // Facing forward
        SwerveModuleState desiredState = new SwerveModuleState(2.0, new Rotation2d(Math.PI)); // Facing backward
        
        SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, currentAngle);
        
        // Optimized state should reverse speed and not turn as far
        assertEquals("Speed should be reversed", -2.0, optimized.speedMetersPerSecond, 0.01);
        assertTrue("Angle should be closer to current angle", 
            Math.abs(optimized.angle.getRadians()) < Math.abs(desiredState.angle.getRadians()));
        
        System.out.println("Module optimization test passed!");
    }

    /**
     * Test basic odometry functionality
     */
    @Test
    public void testOdometry() {
        System.out.println("=== Testing SwerveDriveOdometry ===");
        
        SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics, initialGyro, initialPositions, new Pose2d());
        
        assertEquals("Initial X should be 0", 0.0, odometry.getPoseMeters().getX(), 0.01);
        assertEquals("Initial Y should be 0", 0.0, odometry.getPoseMeters().getY(), 0.01);
        
        // Simulate forward motion for 1 meter
        SwerveModulePosition[] newPositions = {
            new SwerveModulePosition(1.0, new Rotation2d(0)),
            new SwerveModulePosition(1.0, new Rotation2d(0)),
            new SwerveModulePosition(1.0, new Rotation2d(0)),
            new SwerveModulePosition(1.0, new Rotation2d(0))
        };
        
        Pose2d newPose = odometry.update(initialGyro, newPositions);
        assertTrue("Should have moved forward", newPose.getX() > 0.5);
        
        System.out.println("Odometry test passed!");
    }

    /**
     * Test pose estimator with vision
     */
    @Test
    public void testPoseEstimator() {
        System.out.println("=== Testing SwerveDrivePoseEstimator ===");
        
        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, initialGyro, initialPositions, new Pose2d());
        
        assertNotNull("Pose estimator should initialize", poseEstimator.getEstimatedPosition());
        
        // Simulate some motion
        SwerveModulePosition[] newPositions = {
            new SwerveModulePosition(0.5, new Rotation2d(0)),
            new SwerveModulePosition(0.5, new Rotation2d(0)),
            new SwerveModulePosition(0.5, new Rotation2d(0)),
            new SwerveModulePosition(0.5, new Rotation2d(0))
        };
        
        Pose2d estimatedPose = poseEstimator.update(initialGyro, newPositions);
        assertTrue("Should have moved forward", estimatedPose.getX() > 0.1);
        
        // Add a vision measurement
        Pose2d visionPose = new Pose2d(0.6, 0.1, new Rotation2d(Math.toRadians(5)));
        poseEstimator.addVisionMeasurement(visionPose, System.currentTimeMillis() / 1000.0);
        
        Pose2d finalPose = poseEstimator.getEstimatedPosition();
        assertNotNull("Final pose should not be null", finalPose);
        
        System.out.println("Pose estimator test passed!");
    }

    /**
     * Integration test that runs multiple components together
     */
    @Test
    public void testFullIntegration() {
        System.out.println("=== Integration Test ===");
        
        // Test full workflow: kinematics -> odometry -> pose estimation
        ChassisSpeeds testSpeeds = new ChassisSpeeds(1.0, 0.5, 0.2);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(testSpeeds);
        
        // Verify kinematics work
        assertNotNull("Module states should not be null", moduleStates);
        assertEquals("Should have 4 module states", 4, moduleStates.length);
        
        // Create pose estimator
        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, initialGyro, initialPositions, new Pose2d());
        
        // Simulate motion over time
        for (int i = 1; i <= 10; i++) {
            SwerveModulePosition[] positions = new SwerveModulePosition[]{
                new SwerveModulePosition(i * 0.1, new Rotation2d(0)),
                new SwerveModulePosition(i * 0.1, new Rotation2d(0)),
                new SwerveModulePosition(i * 0.1, new Rotation2d(0)),
                new SwerveModulePosition(i * 0.1, new Rotation2d(0))
            };
            
            poseEstimator.update(initialGyro, positions);
        }
        
        Pose2d finalPose = poseEstimator.getEstimatedPosition();
        assertTrue("Robot should have moved forward significantly", finalPose.getX() > 0.5);
        
        System.out.println("Integration test passed!");
    }
}