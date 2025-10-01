package org.firstinspires.ftc.teamcode.Subsystems;

import static org.junit.Assert.*;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests for SwerveDrivePoseEstimator
 */
public class SwervePoseEstimatorTest {

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModulePosition[] modulePositions;

    @Before
    public void setUp() {
        // Create swerve module positions (front-left, front-right, back-left, back-right)
        Translation2d[] moduleLocations = {
            new Translation2d(0.5, 0.5),   // Front Left
            new Translation2d(0.5, -0.5),  // Front Right
            new Translation2d(-0.5, 0.5),  // Back Left
            new Translation2d(-0.5, -0.5)  // Back Right
        };

        kinematics = new SwerveDriveKinematics(moduleLocations);

        // Initialize module positions at zero
        modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(0.0, new Rotation2d(0.0));
        }

        // Create pose estimator starting at origin
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(0.0),
            modulePositions,
            new Pose2d(0.0, 0.0, new Rotation2d(0.0))
        );
    }

    @Test
    public void testInitialPose() {
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        
        assertEquals(0.0, currentPose.getX(), 0.001);
        assertEquals(0.0, currentPose.getY(), 0.001);
        assertEquals(0.0, currentPose.getRotation().getRadians(), 0.001);
    }

    @Test
    public void testOdometryUpdate() {
        // Simulate forward movement
        SwerveModulePosition[] newPositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            newPositions[i] = new SwerveModulePosition(1.0, new Rotation2d(0.0)); // 1 meter forward
        }

        poseEstimator.update(new Rotation2d(0.0), newPositions);
        
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        
        // Should have moved forward by approximately 1 meter
        assertTrue("X position should be positive", currentPose.getX() > 0.5);
        assertEquals(0.0, currentPose.getY(), 0.1); // Should stay on same Y
        assertEquals(0.0, currentPose.getRotation().getRadians(), 0.1); // Should stay same rotation
    }

    @Test
    public void testResetPosition() {
        Pose2d newPose = new Pose2d(1.0, 2.0, new Rotation2d(Math.PI / 4));
        Rotation2d newGyroAngle = new Rotation2d(Math.PI / 4);
        
        poseEstimator.resetPosition(newGyroAngle, modulePositions, newPose);
        
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        assertEquals(1.0, currentPose.getX(), 0.001);
        assertEquals(2.0, currentPose.getY(), 0.001);
        assertEquals(Math.PI / 4, currentPose.getRotation().getRadians(), 0.001);
    }

    @Test
    public void testCreateMatrixUtility() {
        double[] values = {0.1, 0.2, 0.3};
        org.ejml.simple.SimpleMatrix matrix = SwerveDrivePoseEstimator.createMatrix(values);
        
        assertNotNull(matrix);
        assertEquals(3, matrix.getNumElements());
        assertEquals(0.1, matrix.get(0, 0), 0.001);
        assertEquals(0.2, matrix.get(1, 0), 0.001);
        assertEquals(0.3, matrix.get(2, 0), 0.001);
    }
}