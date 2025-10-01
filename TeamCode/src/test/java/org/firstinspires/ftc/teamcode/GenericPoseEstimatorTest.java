package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Subsystems.GenericPoseEstimator;
import org.firstinspires.ftc.teamcode.Subsystems.VisionStandardDeviationCalculator;
import org.junit.Test;
import org.junit.Before;
import static org.junit.Assert.*;

/**
 * JUnit test class to validate the GenericPoseEstimator implementation.
 * 
 * Tests the core functionality including:
 * - Basic pose tracking with odometry updates
 * - Vision measurement integration with latency compensation
 * - Dynamic standard deviation calculation
 * - Historical pose buffering and interpolation
 * - Edge cases and error handling
 * 
 * Run with: ./gradlew :TeamCode:test --tests GenericPoseEstimatorTest
 */
public class GenericPoseEstimatorTest {

    private GenericPoseEstimator poseEstimator;
    private VisionStandardDeviationCalculator visionStdDevCalc;
    private Pose2d initialPose;
    private double testStartTime;

    @Before
    public void setUp() {
        initialPose = new Pose2d(0, 0, new Rotation2d(0));
        
        // Create estimator with moderate trust in both odometry and vision
        SimpleMatrix odometryStdDevs = GenericPoseEstimator.createVector(0.1, 0.1, 0.1);
        SimpleMatrix visionStdDevs = GenericPoseEstimator.createVector(0.3, 0.3, 0.3);
        
        poseEstimator = new GenericPoseEstimator(initialPose, odometryStdDevs, visionStdDevs);
        visionStdDevCalc = new VisionStandardDeviationCalculator();
        
        testStartTime = getCurrentTime();
    }

    /**
     * Test basic initialization and pose retrieval
     */
    @Test
    public void testInitialization() {
        System.out.println("=== Testing GenericPoseEstimator Initialization ===");
        
        Pose2d estimatedPose = poseEstimator.getEstimatedPose();
        
        assertNotNull("Estimated pose should not be null", estimatedPose);
        assertEquals("Initial X should be 0", 0.0, estimatedPose.getX(), 0.001);
        assertEquals("Initial Y should be 0", 0.0, estimatedPose.getY(), 0.001);
        assertEquals("Initial rotation should be 0", 0.0, estimatedPose.getRotation().getRadians(), 0.001);
        
        System.out.println("Initialization test passed!");
    }

    /**
     * Test basic odometry updates with incremental motion
     */
    @Test
    public void testOdometryUpdates() {
        System.out.println("=== Testing Odometry Updates ===");
        
        // Simulate forward motion over 10 steps
        for (int step = 1; step <= 10; step++) {
            double x = step * 0.1; // Move 0.1m forward each step
            Pose2d odometryPose = new Pose2d(x, 0, new Rotation2d(0));
            
            Pose2d estimatedPose = poseEstimator.update(odometryPose);
            
            // Should track odometry closely when no vision corrections
            assertEquals("Step " + step + ": X should match odometry", 
                x, estimatedPose.getX(), 0.02);
            assertEquals("Step " + step + ": Y should stay at 0", 
                0.0, estimatedPose.getY(), 0.02);
            assertEquals("Step " + step + ": Heading should stay at 0", 
                0.0, estimatedPose.getRotation().getRadians(), 0.02);
            
            System.out.printf("Step %d: Odometry=(%.2f, %.2f, %.1f°) Estimated=(%.2f, %.2f, %.1f°)%n",
                step, x, 0.0, 0.0,
                estimatedPose.getX(), estimatedPose.getY(), 
                Math.toDegrees(estimatedPose.getRotation().getRadians()));
        }
        
        System.out.println("Odometry updates test passed!");
    }

    /**
     * Test vision measurement integration without latency
     */
    @Test
    public void testVisionIntegration() {
        System.out.println("=== Testing Vision Integration ===");
        
        // Move robot with odometry
        Pose2d odometryPose = new Pose2d(1.0, 0.0, new Rotation2d(0));
        poseEstimator.update(odometryPose);
        
        Pose2d poseBeforeVision = poseEstimator.getEstimatedPose();
        
        // Add vision measurement that's slightly different from odometry
        Pose2d visionPose = new Pose2d(1.1, 0.05, new Rotation2d(Math.toRadians(2)));
        double visionTime = getCurrentTime();
        poseEstimator.addVisionMeasurement(visionPose, visionTime);
        
        Pose2d poseAfterVision = poseEstimator.getEstimatedPose();
        
        // Pose should be adjusted toward vision measurement
        assertTrue("X should be adjusted toward vision", 
            Math.abs(poseAfterVision.getX() - visionPose.getX()) < 
            Math.abs(poseBeforeVision.getX() - visionPose.getX()));
        
        assertTrue("Y should be adjusted toward vision", 
            Math.abs(poseAfterVision.getY() - visionPose.getY()) < 
            Math.abs(poseBeforeVision.getY() - visionPose.getY()));
        
        System.out.printf("Before vision: (%.3f, %.3f, %.1f°)%n", 
            poseBeforeVision.getX(), poseBeforeVision.getY(), 
            Math.toDegrees(poseBeforeVision.getRotation().getRadians()));
        System.out.printf("Vision measurement: (%.3f, %.3f, %.1f°)%n", 
            visionPose.getX(), visionPose.getY(), 
            Math.toDegrees(visionPose.getRotation().getRadians()));
        System.out.printf("After vision: (%.3f, %.3f, %.1f°)%n", 
            poseAfterVision.getX(), poseAfterVision.getY(), 
            Math.toDegrees(poseAfterVision.getRotation().getRadians()));
        
        System.out.println("Vision integration test passed!");
    }

    /**
     * Test latency compensation with historical pose lookup
     */
    @Test
    public void testLatencyCompensation() {
        System.out.println("=== Testing Latency Compensation ===");
        
        double startTime = getCurrentTime();
        
        // Build up some pose history
        for (int i = 0; i < 10; i++) {
            double time = startTime + i * 0.1; // Every 100ms
            Pose2d odometryPose = new Pose2d(i * 0.1, 0, new Rotation2d(0));
            poseEstimator.updateWithTime(time, odometryPose);
            
            // Small delay to ensure time progression
            try { Thread.sleep(10); } catch (InterruptedException e) {}
        }
        
        // Add vision measurement with latency (timestamp from middle of motion)
        double visionTimestamp = startTime + 0.5; // 500ms ago
        Pose2d visionPose = new Pose2d(0.55, 0.02, new Rotation2d(Math.toRadians(1)));
        
        Pose2d poseBeforeVision = poseEstimator.getEstimatedPose();
        poseEstimator.addVisionMeasurement(visionPose, visionTimestamp);
        Pose2d poseAfterVision = poseEstimator.getEstimatedPose();
        
        // Should still apply correction even with old timestamp
        assertNotEquals("Pose should change after latent vision measurement", 
            poseBeforeVision, poseAfterVision);
        
        System.out.printf("Vision timestamp: %.3f (%.1fms ago)%n", 
            visionTimestamp, (getCurrentTime() - visionTimestamp) * 1000);
        System.out.printf("Pose change: (%.3f, %.3f) -> (%.3f, %.3f)%n",
            poseBeforeVision.getX(), poseBeforeVision.getY(),
            poseAfterVision.getX(), poseAfterVision.getY());
        
        System.out.println("Latency compensation test passed!");
    }

    /**
     * Test rejection of outlier vision measurements
     */
    @Test
    public void testVisionOutlierRejection() {
        System.out.println("=== Testing Vision Outlier Rejection ===");
        
        // Move robot to a known position
        Pose2d odometryPose = new Pose2d(1.0, 0.5, new Rotation2d(0));
        poseEstimator.update(odometryPose);
        
        Pose2d poseBeforeOutlier = poseEstimator.getEstimatedPose();
        
        // Try to add a vision measurement that's way off (should be rejected)
        Pose2d outlierVision = new Pose2d(10.0, 10.0, new Rotation2d(Math.PI));
        poseEstimator.addVisionMeasurement(outlierVision, getCurrentTime());
        
        Pose2d poseAfterOutlier = poseEstimator.getEstimatedPose();
        
        // Pose should not change significantly (outlier rejected)
        double positionChange = poseAfterOutlier.getTranslation()
            .getDistance(poseBeforeOutlier.getTranslation());
        
        assertTrue("Outlier vision measurement should be rejected", positionChange < 0.1);
        
        System.out.printf("Position change from outlier: %.3f m (should be < 0.1m)%n", positionChange);
        System.out.println("Outlier rejection test passed!");
    }

    /**
     * Test custom vision standard deviations
     */
    @Test
    public void testCustomVisionStandardDeviations() {
        System.out.println("=== Testing Custom Vision Standard Deviations ===");
        
        // Move robot
        Pose2d odometryPose = new Pose2d(1.0, 0, new Rotation2d(0));
        poseEstimator.update(odometryPose);
        
        // Add vision measurement with high confidence (low std dev)
        Pose2d highConfidenceVision = new Pose2d(1.05, 0.02, new Rotation2d(0));
        SimpleMatrix highConfidenceStdDev = GenericPoseEstimator.createVector(0.01, 0.01, 0.01);
        
        Pose2d poseBefore = poseEstimator.getEstimatedPose();
        poseEstimator.addVisionMeasurement(highConfidenceVision, getCurrentTime(), highConfidenceStdDev);
        Pose2d poseAfterHighConfidence = poseEstimator.getEstimatedPose();
        
        // Reset and try with low confidence (high std dev)
        poseEstimator.resetPose(odometryPose);
        
        Pose2d lowConfidenceVision = new Pose2d(1.05, 0.02, new Rotation2d(0));
        SimpleMatrix lowConfidenceStdDev = GenericPoseEstimator.createVector(1.0, 1.0, 1.0);
        
        poseEstimator.addVisionMeasurement(lowConfidenceVision, getCurrentTime(), lowConfidenceStdDev);
        Pose2d poseAfterLowConfidence = poseEstimator.getEstimatedPose();
        
        // High confidence vision should have more effect
        double highConfidenceChange = poseAfterHighConfidence.getTranslation()
            .getDistance(poseBefore.getTranslation());
        double lowConfidenceChange = poseAfterLowConfidence.getTranslation()
            .getDistance(poseBefore.getTranslation());
        
        assertTrue("High confidence vision should have more effect", 
            highConfidenceChange > lowConfidenceChange);
        
        System.out.printf("High confidence change: %.4f m%n", highConfidenceChange);
        System.out.printf("Low confidence change: %.4f m%n", lowConfidenceChange);
        System.out.println("Custom standard deviations test passed!");
    }

    /**
     * Test pose reset functionality
     */
    @Test
    public void testPoseReset() {
        System.out.println("=== Testing Pose Reset ===");
        
        // Move robot away from origin
        Pose2d odometryPose = new Pose2d(2.0, 1.5, new Rotation2d(Math.toRadians(45)));
        poseEstimator.update(odometryPose);
        
        // Verify we're not at origin
        Pose2d poseBeforeReset = poseEstimator.getEstimatedPose();
        assertNotEquals("Should not be at origin", 0.0, poseBeforeReset.getX(), 0.1);
        
        // Reset to a known pose
        Pose2d resetPose = new Pose2d(0.5, 0.3, new Rotation2d(Math.toRadians(10)));
        poseEstimator.resetPose(resetPose);
        
        Pose2d poseAfterReset = poseEstimator.getEstimatedPose();
        
        // Should match reset pose exactly
        assertEquals("X should match reset pose", resetPose.getX(), poseAfterReset.getX(), 0.001);
        assertEquals("Y should match reset pose", resetPose.getY(), poseAfterReset.getY(), 0.001);
        assertEquals("Rotation should match reset pose", 
            resetPose.getRotation().getRadians(), 
            poseAfterReset.getRotation().getRadians(), 0.001);
        
        System.out.println("Pose reset test passed!");
    }

    /**
     * Test VisionStandardDeviationCalculator functionality
     */
    @Test
    public void testVisionStandardDeviationCalculator() {
        System.out.println("=== Testing Vision Standard Deviation Calculator ===");
        
        // Test distance-based scaling
        SimpleMatrix closeStdDevs = visionStdDevCalc.calculateStdDevsFromDistance(1.0);
        SimpleMatrix farStdDevs = visionStdDevCalc.calculateStdDevsFromDistance(4.0);
        
        // Far measurements should have higher uncertainty
        assertTrue("Far measurements should have higher X std dev", 
            farStdDevs.get(0, 0) > closeStdDevs.get(0, 0));
        assertTrue("Far measurements should have higher Y std dev", 
            farStdDevs.get(1, 0) > closeStdDevs.get(1, 0));
        assertTrue("Far measurements should have higher rotation std dev", 
            farStdDevs.get(2, 0) > closeStdDevs.get(2, 0));
        
        // Test multi-tag bonus
        SimpleMatrix singleTagStdDevs = visionStdDevCalc.calculateStdDevsFromDistance(2.0, 1.0, 0.0, 1);
        SimpleMatrix multiTagStdDevs = visionStdDevCalc.calculateStdDevsFromDistance(2.0, 1.0, 0.0, 3);
        
        // Multiple tags should improve accuracy (lower std dev)
        assertTrue("Multiple tags should improve X accuracy", 
            multiTagStdDevs.get(0, 0) < singleTagStdDevs.get(0, 0));
        
        // Test confidence-based calculation
        SimpleMatrix highConfidenceStdDevs = visionStdDevCalc.calculateStdDevsFromConfidence(0.9);
        SimpleMatrix lowConfidenceStdDevs = visionStdDevCalc.calculateStdDevsFromConfidence(0.3);
        
        // Low confidence should have higher uncertainty
        assertTrue("Low confidence should have higher std dev", 
            lowConfidenceStdDevs.get(0, 0) > highConfidenceStdDevs.get(0, 0));
        
        System.out.printf("Close (1m): σ=(%.3f, %.3f, %.3f)%n", 
            closeStdDevs.get(0,0), closeStdDevs.get(1,0), closeStdDevs.get(2,0));
        System.out.printf("Far (4m): σ=(%.3f, %.3f, %.3f)%n", 
            farStdDevs.get(0,0), farStdDevs.get(1,0), farStdDevs.get(2,0));
        System.out.printf("Single tag: σ=(%.3f, %.3f, %.3f)%n", 
            singleTagStdDevs.get(0,0), singleTagStdDevs.get(1,0), singleTagStdDevs.get(2,0));
        System.out.printf("Multi tag: σ=(%.3f, %.3f, %.3f)%n", 
            multiTagStdDevs.get(0,0), multiTagStdDevs.get(1,0), multiTagStdDevs.get(2,0));
        
        System.out.println("Vision standard deviation calculator test passed!");
    }

    /**
     * Test edge cases and error conditions
     */
    @Test
    public void testEdgeCases() {
        System.out.println("=== Testing Edge Cases ===");
        
        // Test adding vision measurement before any odometry updates
        GenericPoseEstimator freshEstimator = new GenericPoseEstimator(new Pose2d());
        Pose2d visionPose = new Pose2d(0.1, 0.1, new Rotation2d(0));
        
        // Should handle gracefully (no crash)
        freshEstimator.addVisionMeasurement(visionPose, getCurrentTime());
        assertNotNull("Should still have valid pose", freshEstimator.getEstimatedPose());
        
        // Test very old vision measurements (outside buffer)
        poseEstimator.update(new Pose2d(1, 0, new Rotation2d(0)));
        
        // Add measurement with very old timestamp
        double oldTimestamp = getCurrentTime() - 5.0; // 5 seconds ago
        poseEstimator.addVisionMeasurement(new Pose2d(1.1, 0, new Rotation2d(0)), oldTimestamp);
        
        // Should handle gracefully
        assertNotNull("Should handle old timestamps", poseEstimator.getEstimatedPose());
        
        // Test zero standard deviations (edge case)
        SimpleMatrix zeroStdDevs = GenericPoseEstimator.createVector(0.0, 0.0, 0.0);
        poseEstimator.addVisionMeasurement(visionPose, getCurrentTime(), zeroStdDevs);
        
        // Should handle gracefully
        assertNotNull("Should handle zero std devs", poseEstimator.getEstimatedPose());
        
        System.out.println("Edge cases test passed!");
    }

    /**
     * Integration test combining multiple features
     */
    @Test
    public void testFullIntegration() {
        System.out.println("=== Testing Full Integration ===");
        
        double startTime = getCurrentTime();
        
        // Simulate realistic robot motion with periodic vision updates
        for (int i = 0; i < 20; i++) {
            double time = startTime + i * 0.1;
            
            // Robot moves in a circle
            double x = 1.0 + 0.5 * Math.cos(i * 0.2);
            double y = 1.0 + 0.5 * Math.sin(i * 0.2);
            double theta = i * 0.2;
            
            Pose2d odometryPose = new Pose2d(x, y, new Rotation2d(theta));
            poseEstimator.updateWithTime(time, odometryPose);
            
            // Add vision measurement every 5 steps with some noise and latency
            if (i % 5 == 0 && i > 0) {
                double visionLatency = 0.15; // 150ms latency
                double visionTimestamp = time - visionLatency;
                
                // Vision measurement with small error
                Pose2d visionPose = new Pose2d(
                    x + (Math.random() - 0.5) * 0.1, 
                    y + (Math.random() - 0.5) * 0.1, 
                    new Rotation2d(theta + (Math.random() - 0.5) * 0.1)
                );
                
                // Calculate dynamic standard deviations based on distance
                double distanceToOrigin = Math.sqrt(x*x + y*y);
                SimpleMatrix visionStdDevs = visionStdDevCalc.calculateStdDevsFromDistance(distanceToOrigin);
                
                poseEstimator.addVisionMeasurement(visionPose, visionTimestamp, visionStdDevs);
                
                System.out.printf("Step %d: Vision update at distance %.2fm%n", i, distanceToOrigin);
            }
            
            // Small delay to simulate real timing
            try { Thread.sleep(5); } catch (InterruptedException e) {}
        }
        
        Pose2d finalPose = poseEstimator.getEstimatedPose();
        
        // Should have a reasonable final pose
        assertNotNull("Final pose should not be null", finalPose);
        assertTrue("X should be reasonable", Math.abs(finalPose.getX()) < 10.0);
        assertTrue("Y should be reasonable", Math.abs(finalPose.getY()) < 10.0);
        
        System.out.printf("Final estimated pose: (%.3f, %.3f, %.1f°)%n", 
            finalPose.getX(), finalPose.getY(), 
            Math.toDegrees(finalPose.getRotation().getRadians()));
        
        System.out.println("Full integration test passed!");
    }

    /**
     * Helper method to get current time in seconds
     */
    private double getCurrentTime() {
        return System.nanoTime() / 1e9;
    }
}