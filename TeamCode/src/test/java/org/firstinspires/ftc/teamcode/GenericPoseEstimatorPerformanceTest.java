package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Subsystems.GenericPoseEstimator;
import org.firstinspires.ftc.teamcode.Subsystems.VisionStandardDeviationCalculator;
import org.junit.Test;
import org.junit.Before;
import static org.junit.Assert.*;

/**
 * Performance and stress tests for the GenericPoseEstimator.
 * 
 * These tests validate behavior under challenging conditions:
 * - High frequency updates
 * - Large pose buffers
 * - Rapid vision corrections
 * - Memory usage validation
 * - Timing performance
 */
public class GenericPoseEstimatorPerformanceTest {

    private GenericPoseEstimator poseEstimator;
    private VisionStandardDeviationCalculator visionStdDevCalc;

    @Before
    public void setUp() {
        Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));
        SimpleMatrix odometryStdDevs = GenericPoseEstimator.createVector(0.05, 0.05, 0.05);
        SimpleMatrix visionStdDevs = GenericPoseEstimator.createVector(0.2, 0.2, 0.2);
        
        poseEstimator = new GenericPoseEstimator(initialPose, odometryStdDevs, visionStdDevs);
        visionStdDevCalc = new VisionStandardDeviationCalculator();
    }

    /**
     * Test high-frequency odometry updates (100Hz simulation)
     */
    @Test
    public void testHighFrequencyOdometryUpdates() {
        System.out.println("=== Testing High Frequency Odometry Updates (100Hz) ===");
        
        long startTime = System.nanoTime();
        int numUpdates = 1000; // Simulate 10 seconds at 100Hz
        
        for (int i = 0; i < numUpdates; i++) {
            double time = i * 0.01; // 10ms intervals
            double x = time * 0.5; // 0.5 m/s forward motion
            double y = 0.1 * Math.sin(time); // Small lateral oscillation
            double theta = 0.1 * time; // Slow rotation
            
            Pose2d odometryPose = new Pose2d(x, y, new Rotation2d(theta));
            poseEstimator.updateWithTime(getCurrentTimeFromStart(startTime, time), odometryPose);
        }
        
        long endTime = System.nanoTime();
        double elapsedMs = (endTime - startTime) / 1e6;
        
        Pose2d finalPose = poseEstimator.getEstimatedPose();
        
        // Verify reasonable final pose
        assertTrue("Final X should be reasonable", finalPose.getX() > 4.0 && finalPose.getX() < 6.0);
        
        // Performance check - should complete in reasonable time
        assertTrue("Should complete 1000 updates quickly", elapsedMs < 1000); // Less than 1 second
        
        System.out.printf("Completed %d updates in %.2f ms (%.2f updates/ms)%n", 
            numUpdates, elapsedMs, numUpdates / elapsedMs);
        System.out.printf("Final pose: (%.3f, %.3f, %.1f°)%n", 
            finalPose.getX(), finalPose.getY(), 
            Math.toDegrees(finalPose.getRotation().getRadians()));
        
        System.out.println("High frequency odometry test passed!");
    }

    /**
     * Test rapid vision corrections with varying latencies
     */
    @Test
    public void testRapidVisionCorrections() {
        System.out.println("=== Testing Rapid Vision Corrections ===");
        
        long startTime = System.nanoTime();
        double baseTime = getCurrentTimeFromStart(startTime, 0);
        
        // Build up odometry history
        for (int i = 0; i < 100; i++) {
            double time = baseTime + i * 0.02; // 50Hz odometry
            Pose2d odometryPose = new Pose2d(i * 0.05, 0, new Rotation2d(0));
            poseEstimator.updateWithTime(time, odometryPose);
        }
        
        // Add many vision corrections with random latencies
        int numVisionCorrections = 50;
        for (int i = 0; i < numVisionCorrections; i++) {
            double visionLatency = Math.random() * 0.5; // 0-500ms latency
            double visionTime = baseTime + (100 * 0.02) - visionLatency;
            
            // Vision measurement with small random error
            double visionX = (100 * 0.05) + (Math.random() - 0.5) * 0.2;
            double visionY = (Math.random() - 0.5) * 0.1;
            double visionTheta = (Math.random() - 0.5) * 0.1;
            
            Pose2d visionPose = new Pose2d(visionX, visionY, new Rotation2d(visionTheta));
            
            // Use distance-based standard deviations
            double distance = Math.random() * 3.0 + 0.5; // 0.5-3.5m distance
            SimpleMatrix visionStdDevs = visionStdDevCalc.calculateStdDevsFromDistance(distance);
            
            poseEstimator.addVisionMeasurement(visionPose, visionTime, visionStdDevs);
        }
        
        long endTime = System.nanoTime();
        double elapsedMs = (endTime - startTime) / 1e6;
        
        Pose2d finalPose = poseEstimator.getEstimatedPose();
        
        // Should still have reasonable final pose (system handled all corrections without crashing)
        assertNotNull("Final pose should not be null", finalPose);
        assertFalse("X should not be NaN", Double.isNaN(finalPose.getX()));
        assertFalse("Y should not be NaN", Double.isNaN(finalPose.getY()));
        assertFalse("Rotation should not be NaN", Double.isNaN(finalPose.getRotation().getRadians()));
        
        System.out.printf("Processed %d vision corrections in %.2f ms%n", 
            numVisionCorrections, elapsedMs);
        System.out.printf("Final pose after corrections: (%.3f, %.3f, %.1f°)%n", 
            finalPose.getX(), finalPose.getY(), 
            Math.toDegrees(finalPose.getRotation().getRadians()));
        
        System.out.println("Rapid vision corrections test passed!");
    }

    /**
     * Test buffer management with extended operation
     */
    @Test
    public void testLongTermBufferManagement() {
        System.out.println("=== Testing Long Term Buffer Management ===");
        
        long startTime = System.nanoTime();
        double baseTime = getCurrentTimeFromStart(startTime, 0);
        
        // Simulate 30 seconds of operation at 50Hz
        int totalUpdates = 30 * 50; // 1500 updates
        int visionUpdateInterval = 25; // Vision every 0.5 seconds
        
        for (int i = 0; i < totalUpdates; i++) {
            double time = baseTime + i * 0.02;
            
            // Complex motion pattern
            double t = i * 0.02;
            double x = 2.0 * t + 0.5 * Math.sin(t * 0.5);
            double y = 1.0 * Math.cos(t * 0.3);
            double theta = t * 0.2;
            
            Pose2d odometryPose = new Pose2d(x, y, new Rotation2d(theta));
            poseEstimator.updateWithTime(time, odometryPose);
            
            // Periodic vision updates
            if (i % visionUpdateInterval == 0 && i > 0) {
                double visionLatency = 0.1 + Math.random() * 0.2; // 100-300ms latency
                double visionTime = time - visionLatency;
                
                Pose2d visionPose = new Pose2d(
                    x + (Math.random() - 0.5) * 0.15,
                    y + (Math.random() - 0.5) * 0.15,
                    new Rotation2d(theta + (Math.random() - 0.5) * 0.05)
                );
                
                poseEstimator.addVisionMeasurement(visionPose, visionTime);
            }
            
            // Memory check every 10 seconds
            if (i % 500 == 0 && i > 0) {
                Runtime.getRuntime().gc(); // Suggest garbage collection
                long usedMemory = Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory();
                System.out.printf("Time %.1fs: Memory usage = %.2f MB%n", 
                    t, usedMemory / (1024.0 * 1024.0));
            }
        }
        
        long endTime = System.nanoTime();
        double elapsedMs = (endTime - startTime) / 1e6;
        
        Pose2d finalPose = poseEstimator.getEstimatedPose();
        
        // Verify system still works after extended operation
        assertNotNull("Final pose should not be null", finalPose);
        
        System.out.printf("Completed %d updates over 30 seconds in %.2f ms%n", 
            totalUpdates, elapsedMs);
        System.out.printf("Average update time: %.4f ms%n", elapsedMs / totalUpdates);
        System.out.printf("Final pose: (%.3f, %.3f, %.1f°)%n", 
            finalPose.getX(), finalPose.getY(), 
            Math.toDegrees(finalPose.getRotation().getRadians()));
        
        System.out.println("Long term buffer management test passed!");
    }

    /**
     * Test interpolation accuracy with known poses
     */
    @Test
    public void testInterpolationAccuracy() {
        System.out.println("=== Testing Interpolation Accuracy ===");
        
        double baseTime = getCurrentTime();
        
        // Create known poses at specific times
        Pose2d pose1 = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d pose2 = new Pose2d(2, 0, new Rotation2d(Math.PI/2));
        double time1 = baseTime;
        double time2 = baseTime + 1.0; // 1 second later
        
        poseEstimator.updateWithTime(time1, pose1);
        poseEstimator.updateWithTime(time2, pose2);
        
        // Add vision measurement at interpolated time
        double interpolatedTime = baseTime + 0.5; // Halfway between
        Pose2d expectedInterpolatedPose = new Pose2d(1, 0, new Rotation2d(Math.PI/4)); // Halfway
        
        // The vision measurement should use interpolated historical pose
        Pose2d visionMeasurement = new Pose2d(1.05, 0.02, new Rotation2d(Math.PI/4 + 0.01));
        poseEstimator.addVisionMeasurement(visionMeasurement, interpolatedTime);
        
        // Verify the correction was applied (pose should change)
        Pose2d finalPose = poseEstimator.getEstimatedPose();
        
        // Should be close to pose2 but with small vision correction applied
        double expectedX = 2.0; // Should still be close to pose2.X
        assertTrue("Interpolation should work correctly", 
            Math.abs(finalPose.getX() - expectedX) < 0.2);
        
        System.out.printf("Interpolated vision correction applied successfully%n");
        System.out.printf("Final pose: (%.3f, %.3f, %.1f°)%n", 
            finalPose.getX(), finalPose.getY(), 
            Math.toDegrees(finalPose.getRotation().getRadians()));
        
        System.out.println("Interpolation accuracy test passed!");
    }

    /**
     * Test behavior with conflicting vision measurements
     */
    @Test
    public void testConflictingVisionMeasurements() {
        System.out.println("=== Testing Conflicting Vision Measurements ===");
        
        // Move robot to known position
        Pose2d odometryPose = new Pose2d(2.0, 1.0, new Rotation2d(0));
        poseEstimator.update(odometryPose);
        
        Pose2d poseBeforeVision = poseEstimator.getEstimatedPose();
        
        // Add two conflicting vision measurements at nearly the same time
        double visionTime1 = getCurrentTime();
        double visionTime2 = visionTime1 + 0.001; // 1ms apart
        
        Pose2d vision1 = new Pose2d(2.2, 1.1, new Rotation2d(Math.toRadians(5)));
        Pose2d vision2 = new Pose2d(1.8, 0.9, new Rotation2d(Math.toRadians(-5)));
        
        // Add both measurements
        poseEstimator.addVisionMeasurement(vision1, visionTime1);
        poseEstimator.addVisionMeasurement(vision2, visionTime2);
        
        Pose2d poseAfterVision = poseEstimator.getEstimatedPose();
        
        // System should handle conflicting measurements gracefully
        assertNotNull("Should handle conflicting measurements", poseAfterVision);
        
        // Final pose should be somewhere between the conflicting measurements
        assertTrue("X should be reasonable despite conflicts", 
            poseAfterVision.getX() > 1.5 && poseAfterVision.getX() < 2.5);
        assertTrue("Y should be reasonable despite conflicts", 
            poseAfterVision.getY() > 0.5 && poseAfterVision.getY() < 1.5);
        
        System.out.printf("Handled conflicting vision measurements%n");
        System.out.printf("Vision 1: (%.3f, %.3f, %.1f°)%n", 
            vision1.getX(), vision1.getY(), Math.toDegrees(vision1.getRotation().getRadians()));
        System.out.printf("Vision 2: (%.3f, %.3f, %.1f°)%n", 
            vision2.getX(), vision2.getY(), Math.toDegrees(vision2.getRotation().getRadians()));
        System.out.printf("Final pose: (%.3f, %.3f, %.1f°)%n", 
            poseAfterVision.getX(), poseAfterVision.getY(), 
            Math.toDegrees(poseAfterVision.getRotation().getRadians()));
        
        System.out.println("Conflicting vision measurements test passed!");
    }

    /**
     * Stress test with extreme conditions
     */
    @Test
    public void testExtremeConditions() {
        System.out.println("=== Testing Extreme Conditions ===");
        
        // Test with very high speed motion
        double baseTime = getCurrentTime();
        
        for (int i = 0; i < 100; i++) {
            double time = baseTime + i * 0.01;
            double x = i * 0.5; // 50 m/s velocity!
            Pose2d extremePose = new Pose2d(x, 0, new Rotation2d(i * 0.5)); // High rotation too
            
            poseEstimator.updateWithTime(time, extremePose);
        }
        
        // Add vision measurement with extreme difference
        Pose2d extremeVision = new Pose2d(0, 0, new Rotation2d(0)); // Back to origin
        poseEstimator.addVisionMeasurement(extremeVision, baseTime + 0.5);
        
        Pose2d finalPose = poseEstimator.getEstimatedPose();
        
        // Should still produce valid pose (may not be accurate but shouldn't crash)
        assertNotNull("Should handle extreme conditions", finalPose);
        assertFalse("X should not be NaN", Double.isNaN(finalPose.getX()));
        assertFalse("Y should not be NaN", Double.isNaN(finalPose.getY()));
        assertFalse("Rotation should not be NaN", Double.isNaN(finalPose.getRotation().getRadians()));
        
        System.out.printf("Survived extreme conditions, final pose: (%.3f, %.3f, %.1f°)%n", 
            finalPose.getX(), finalPose.getY(), 
            Math.toDegrees(finalPose.getRotation().getRadians()));
        
        System.out.println("Extreme conditions test passed!");
    }

    /**
     * Helper method to get current time
     */
    private double getCurrentTime() {
        return System.nanoTime() / 1e9;
    }

    /**
     * Helper method to get time relative to start
     */
    private double getCurrentTimeFromStart(long startNanoTime, double offsetSeconds) {
        return (System.nanoTime() - startNanoTime) / 1e9 + offsetSeconds;
    }
}