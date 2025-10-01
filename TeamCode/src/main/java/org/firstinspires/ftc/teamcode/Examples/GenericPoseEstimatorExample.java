package org.firstinspires.ftc.teamcode.Examples;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.Subsystems.GenericPoseEstimator;
import org.firstinspires.ftc.teamcode.Subsystems.VisionStandardDeviationCalculator;

import java.util.List;

/**
 * Example OpMode demonstrating the GenericPoseEstimator with AprilTag vision.
 * 
 * This example shows how to:
 * 1. Set up the generic pose estimator with any odometry system
 * 2. Integrate AprilTag vision measurements
 * 3. Use dynamic standard deviations based on measurement quality
 * 4. Handle real-world scenarios like multiple tags, varying distances, etc.
 */
@TeleOp(name = "Generic Pose Estimator Example")
public class GenericPoseEstimatorExample extends LinearOpMode {
    
    // Pose estimation components
    private GenericPoseEstimator poseEstimator;
    private VisionStandardDeviationCalculator visionStdDevCalc;
    
    // Vision components (using FTC's AprilTag processor)
    private AprilTagProcessor aprilTagProcessor;
    
    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private double lastVisionTime = 0;
    private double totalDistanceTraveled = 0;
    private Pose2d lastPose = new Pose2d();
    
    // Known AprilTag positions on the field (example positions)
    private final Pose2d[] FIELD_APRILTAGS = {
        new Pose2d(1.50, 1.50, new Rotation2d(0)),      // Tag 1
        new Pose2d(1.50, 2.50, new Rotation2d(0)),      // Tag 2  
        new Pose2d(2.50, 1.50, new Rotation2d(Math.PI)), // Tag 3
        new Pose2d(2.50, 2.50, new Rotation2d(Math.PI))  // Tag 4
    };
    
    @Override
    public void runOpMode() {
        initializePoseEstimator();
        initializeVision();
        
        telemetry.addLine("Generic Pose Estimator initialized");
        telemetry.addLine("Press start to begin");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            updateOdometry();
            updateVisionMeasurements();
            updateTelemetry();
            
            sleep(20); // 50Hz update rate
        }
    }
    
    private void initializePoseEstimator() {
        // Initialize with a known starting position (could be set by auto or user input)
        Pose2d initialPose = new Pose2d(0.5, 0.5, new Rotation2d(0)); // Start 0.5m from corner
        
        // Configure standard deviations based on your odometry system
        // These values should be tuned for your specific setup
        SimpleMatrix odometryStdDevs = GenericPoseEstimator.createVector(
            0.08,  // X standard deviation in meters (8cm uncertainty)
            0.08,  // Y standard deviation in meters  
            0.06   // Rotation standard deviation in radians (~3.4 degrees)
        );
        
        SimpleMatrix defaultVisionStdDevs = GenericPoseEstimator.createVector(
            0.25,  // X standard deviation for vision (25cm base uncertainty)
            0.25,  // Y standard deviation for vision
            0.25   // Rotation standard deviation for vision (~14 degrees)
        );
        
        poseEstimator = new GenericPoseEstimator(initialPose, odometryStdDevs, defaultVisionStdDevs);
        
        // Configure vision standard deviation calculator
        visionStdDevCalc = new VisionStandardDeviationCalculator();
        visionStdDevCalc.setBaseStandardDeviations(0.1, 0.1, 0.1); // Optimistic base values
        visionStdDevCalc.setDistanceScaling(0.15, 5.0, 0.3); // Error increases with distance
        visionStdDevCalc.setMultiTagBonus(0.6); // 40% improvement when seeing multiple tags
        
        // Set maximum vision distance and enable latency compensation
        poseEstimator.setMaxVisionDistance(4.0); // Reject vision beyond 4 meters
        poseEstimator.setLatencyCompensationEnabled(true);
    }
    
    private void initializeVision() {
        // Initialize AprilTag processor (this would typically be done in your vision subsystem)
        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            // .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) // Set appropriate tag library
            .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
            .build();
    }
    
    private void updateOdometry() {
        // This is where you'd get the pose from your specific odometry system
        // Examples:
        
        // For Pinpoint:
        // pinpoint.update();
        // Pose2d odometryPose = new Pose2d(
        //     pinpoint.getPosX() / 1000.0,
        //     pinpoint.getPosY() / 1000.0, 
        //     new Rotation2d(Math.toRadians(pinpoint.getHeading()))
        // );
        
        // For custom three-wheel odometry:
        // customOdometry.update(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        // Pose2d odometryPose = customOdometry.getRobotPose();
        
        // For this example, we'll simulate odometry (replace with your actual system)
        Pose2d odometryPose = simulateOdometry();
        
        // Update the pose estimator
        poseEstimator.update(odometryPose);
        
        // Track distance traveled for drift compensation
        double distance = odometryPose.getTranslation().getDistance(lastPose.getTranslation());
        totalDistanceTraveled += distance;
        lastPose = odometryPose;
    }
    
    private void updateVisionMeasurements() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        
        if (!detections.isEmpty()) {
            // Process the best detection (closest, most head-on, etc.)
            AprilTagDetection bestDetection = selectBestDetection(detections);
            
            if (bestDetection != null) {
                Pose2d visionPose = calculateRobotPoseFromDetection(bestDetection);
                double timestamp = runtime.seconds() - estimateVisionLatency(bestDetection);
                
                // Calculate dynamic standard deviations
                SimpleMatrix visionStdDevs = calculateDynamicStdDevs(bestDetection, detections.size());
                
                // Add the vision measurement
                poseEstimator.addVisionMeasurement(visionPose, timestamp, visionStdDevs);
                
                lastVisionTime = runtime.seconds();
                totalDistanceTraveled = 0; // Reset drift tracking
                
                telemetry.addLine("✓ Vision measurement added");
                telemetry.addData("Vision Distance", "%.2f m", bestDetection.ftcPose.range);
                telemetry.addData("Tags Visible", detections.size());
            }
        }
    }
    
    private AprilTagDetection selectBestDetection(List<AprilTagDetection> detections) {
        AprilTagDetection best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        
        for (AprilTagDetection detection : detections) {
            // Score based on distance (closer is better) and viewing angle
            double distanceScore = 1.0 / Math.max(0.5, detection.ftcPose.range); // Closer = higher score
            double angleScore = Math.cos(detection.ftcPose.bearing); // More head-on = higher score
            double totalScore = distanceScore * angleScore;
            
            if (totalScore > bestScore) {
                best = detection;
                bestScore = totalScore;
            }
        }
        
        return best;
    }
    
    private Pose2d calculateRobotPoseFromDetection(AprilTagDetection detection) {
        // Get the known field position of this tag
        if (detection.id < 1 || detection.id > FIELD_APRILTAGS.length) {
            return null; // Unknown tag
        }
        
        Pose2d tagFieldPose = FIELD_APRILTAGS[detection.id - 1];
        
        // Convert detection to robot field pose
        // This is a simplified calculation - you may need more sophisticated transforms
        double tagX = tagFieldPose.getX();
        double tagY = tagFieldPose.getY();
        double tagHeading = tagFieldPose.getRotation().getRadians();
        
        // Robot position relative to tag (in tag's coordinate frame)
        double robotX = -detection.ftcPose.x; // Camera sees negative X as robot to the right
        double robotY = -detection.ftcPose.z; // Camera sees negative Z as robot forward
        double robotHeading = tagHeading + detection.ftcPose.yaw;
        
        // Transform to field coordinates
        double fieldX = tagX + robotX * Math.cos(tagHeading) - robotY * Math.sin(tagHeading);
        double fieldY = tagY + robotX * Math.sin(tagHeading) + robotY * Math.cos(tagHeading);
        
        return new Pose2d(fieldX, fieldY, new Rotation2d(robotHeading));
    }
    
    private SimpleMatrix calculateDynamicStdDevs(AprilTagDetection detection, int numTags) {
        double distance = detection.ftcPose.range;
        double viewingAngle = Math.abs(detection.ftcPose.bearing);
        
        // Calculate standard deviations based on measurement quality
        SimpleMatrix stdDevs = visionStdDevCalc.calculateStdDevsFromDistance(
            distance,
            1.0,           // Assume standard tag size
            viewingAngle,
            numTags
        );
        
        // Also consider odometry drift
        double timeSinceLastVision = runtime.seconds() - lastVisionTime;
        SimpleMatrix driftStdDevs = visionStdDevCalc.calculateStdDevsFromOdometryDrift(
            timeSinceLastVision, 
            totalDistanceTraveled
        );
        
        // Use the more optimistic (lower) standard deviations
        SimpleMatrix finalStdDevs = new SimpleMatrix(3, 1);
        for (int i = 0; i < 3; i++) {
            finalStdDevs.set(i, 0, Math.min(stdDevs.get(i, 0), driftStdDevs.get(i, 0)));
        }
        
        return finalStdDevs;
    }
    
    private double estimateVisionLatency(AprilTagDetection detection) {
        // Estimate processing latency based on detection complexity
        // More complex scenes (more tags, farther distance) take longer to process
        double baseLatency = 0.050; // 50ms base latency
        double distanceLatency = detection.ftcPose.range * 0.010; // 10ms per meter
        return baseLatency + distanceLatency;
    }
    
    private Pose2d simulateOdometry() {
        // This simulates odometry for the example - replace with your actual odometry system
        double time = runtime.seconds();
        return new Pose2d(
            1.0 + 0.5 * Math.sin(time * 0.3),
            1.0 + 0.3 * Math.cos(time * 0.2), 
            new Rotation2d(time * 0.1)
        );
    }
    
    private void updateTelemetry() {
        Pose2d estimatedPose = poseEstimator.getEstimatedPose();
        
        telemetry.addLine("=== Generic Pose Estimator ===");
        telemetry.addData("Estimated X", "%.3f m", estimatedPose.getX());
        telemetry.addData("Estimated Y", "%.3f m", estimatedPose.getY());
        telemetry.addData("Estimated Heading", "%.1f°", 
            Math.toDegrees(estimatedPose.getRotation().getRadians()));
        
        telemetry.addLine();
        telemetry.addData("Time since vision", "%.1f s", runtime.seconds() - lastVisionTime);
        telemetry.addData("Distance traveled", "%.2f m", totalDistanceTraveled);
        
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        telemetry.addData("Tags visible", detections.size());
        
        if (!detections.isEmpty()) {
            AprilTagDetection closest = detections.get(0);
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose.range < closest.ftcPose.range) {
                    closest = detection;
                }
            }
            telemetry.addData("Closest tag", "ID %d (%.2f m)", closest.id, closest.ftcPose.range);
        }
        
        telemetry.update();
    }
}