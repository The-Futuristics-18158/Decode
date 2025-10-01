# Generic Pose Estimator Usage Examples

The `GenericPoseEstimator` is designed to work with any odometry system that can provide direct `Pose2d` measurements. Unlike drive-specific estimators that require individual wheel encoder data, this estimator accepts complete pose estimates and fuses them with vision measurements.

## Key Concepts

### 1. Separation of Concerns
- **Odometry System**: Calculates robot pose from sensor data (pods, encoders, IMU)
- **Pose Estimator**: Fuses odometry with vision measurements using Kalman filtering
- **Vision System**: Provides occasional position corrections from AprilTags or other landmarks

### 2. Error Modeling
- **Odometry Error**: Usually grows over time/distance (cumulative drift)
- **Vision Error**: Typically increases with distance from targets, viewing angle, lighting conditions
- **Kalman Filter**: Optimally weights measurements based on their uncertainty

## Usage Examples

### Example 1: goBILDA Pinpoint Odometry Computer

The Pinpoint sensor combines two odometry pods with an IMU to provide direct pose estimates.

```java
// In your OpMode or robot class
GenericPoseEstimator poseEstimator;
GoBildaPinpointDriver pinpoint;
VisionStandardDeviationCalculator visionStdDevCalc;

@Override
public void init() {
    // Initialize Pinpoint sensor
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    pinpoint.setOffsets(-84.0, -168.0); // Set pod offsets in mm
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                 GoBildaPinpointDriver.EncoderDirection.FORWARD);
    pinpoint.resetPosAndIMU();
    
    // Initialize pose estimator
    Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));
    poseEstimator = new GenericPoseEstimator(
        initialPose,
        GenericPoseEstimator.createVector(0.05, 0.05, 0.05),  // Trust Pinpoint fairly well
        GenericPoseEstimator.createVector(0.3, 0.3, 0.3)      // Default vision uncertainty
    );
    
    // Initialize vision standard deviation calculator
    visionStdDevCalc = new VisionStandardDeviationCalculator();
    visionStdDevCalc.setBaseStandardDeviations(0.1, 0.1, 0.1); // Base vision accuracy
}

@Override
public void loop() {
    // Update Pinpoint sensor
    pinpoint.update();
    
    // Get odometry pose (Pinpoint handles the coordinate conversion)
    Pose2d odometryPose = new Pose2d(
        pinpoint.getPosX() / 1000.0,  // Convert mm to meters
        pinpoint.getPosY() / 1000.0,  // Convert mm to meters
        new Rotation2d(Math.toRadians(pinpoint.getHeading()))
    );
    
    // Update pose estimator with odometry
    Pose2d estimatedPose = poseEstimator.update(odometryPose);
    
    // If vision system detects AprilTag, add vision measurement
    if (visionSystem.hasNewMeasurement()) {
        VisionMeasurement measurement = visionSystem.getLatestMeasurement();
        
        // Calculate dynamic standard deviations based on distance to tag
        double distanceToTag = measurement.getDistanceToTag();
        SimpleMatrix visionStdDevs = visionStdDevCalc.calculateStdDevsFromDistance(
            distanceToTag, 
            measurement.getTagSize(),
            measurement.getViewingAngle(),
            measurement.getNumberOfTags()
        );
        
        poseEstimator.addVisionMeasurement(
            measurement.getRobotPose(),
            measurement.getTimestamp(),
            visionStdDevs
        );
    }
    
    // Use the estimated pose for path following, etc.
    telemetry.addData("Estimated X", estimatedPose.getX());
    telemetry.addData("Estimated Y", estimatedPose.getY());
    telemetry.addData("Estimated Heading", Math.toDegrees(estimatedPose.getRotation().getRadians()));
}
```

### Example 2: REV Odometry System

Using REV's new odometry computer or custom odometry pod setup.

```java
// In your OpMode or robot class
GenericPoseEstimator poseEstimator;
RevOdometrySystem revOdometry; // Hypothetical REV system
VisionStandardDeviationCalculator visionStdDevCalc;

@Override
public void init() {
    // Initialize REV odometry system
    revOdometry = new RevOdometrySystem(hardwareMap);
    revOdometry.setWheelbaseWidth(0.381); // 15 inches in meters
    revOdometry.setWheelbaseLength(0.381); // 15 inches in meters
    revOdometry.reset();
    
    // Initialize pose estimator with appropriate trust levels
    Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));
    poseEstimator = new GenericPoseEstimator(
        initialPose,
        GenericPoseEstimator.createVector(0.08, 0.08, 0.06),  // REV system uncertainties
        GenericPoseEstimator.createVector(0.25, 0.25, 0.25)   // Vision uncertainties
    );
    
    visionStdDevCalc = new VisionStandardDeviationCalculator();
}

@Override
public void loop() {
    // Update REV odometry
    revOdometry.update();
    
    // Get pose from REV system
    Pose2d odometryPose = revOdometry.getRobotPose();
    
    // Update estimator
    Pose2d estimatedPose = poseEstimator.update(odometryPose);
    
    // Handle vision measurements (similar to Pinpoint example)
    handleVisionMeasurements();
}
```

### Example 3: Custom Odometry Pods with OctoQuad

For teams using custom odometry setups with multiple encoders.

```java
public class CustomOdometrySystem {
    private OctoQuad octoQuad;
    private double lastLeftPos = 0, lastRightPos = 0, lastPerpendicularPos = 0;
    private double lastHeading = 0;
    private Pose2d currentPose = new Pose2d();
    
    // Odometry constants
    private static final double WHEEL_RADIUS = 0.024; // 24mm wheels in meters  
    private static final double TICKS_PER_REV = 2000;
    private static final double DISTANCE_PER_TICK = 2 * Math.PI * WHEEL_RADIUS / TICKS_PER_REV;
    private static final double LATERAL_DISTANCE = 0.350; // Distance between parallel wheels
    private static final double FORWARD_OFFSET = 0.100;   // Perpendicular wheel offset
    
    public void update(double currentHeading) {
        // Read encoder positions
        double leftPos = octoQuad.readChannel(0) * DISTANCE_PER_TICK;
        double rightPos = octoQuad.readChannel(1) * DISTANCE_PER_TICK;
        double perpPos = octoQuad.readChannel(2) * DISTANCE_PER_TICK;
        
        // Calculate deltas
        double deltaLeft = leftPos - lastLeftPos;
        double deltaRight = rightPos - lastRightPos;
        double deltaPerp = perpPos - lastPerpendicularPos;
        double deltaHeading = currentHeading - lastHeading;
        
        // Three-wheel odometry algorithm
        double deltaForward = (deltaLeft + deltaRight) / 2.0;
        double deltaStrafe = deltaPerp - FORWARD_OFFSET * deltaHeading;
        
        // Update pose
        double avgHeading = lastHeading + deltaHeading / 2.0;
        double deltaX = deltaForward * Math.cos(avgHeading) - deltaStrafe * Math.sin(avgHeading);
        double deltaY = deltaForward * Math.sin(avgHeading) + deltaStrafe * Math.cos(avgHeading);
        
        currentPose = new Pose2d(
            currentPose.getX() + deltaX,
            currentPose.getY() + deltaY,
            new Rotation2d(currentHeading)
        );
        
        // Update last positions
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        lastPerpendicularPos = perpPos;
        lastHeading = currentHeading;
    }
    
    public Pose2d getRobotPose() {
        return currentPose;
    }
}

// In your OpMode
GenericPoseEstimator poseEstimator;
CustomOdometrySystem customOdometry;
IMU imu;

@Override
public void init() {
    customOdometry = new CustomOdometrySystem();
    imu = hardwareMap.get(IMU.class, "imu");
    
    // Custom odometry might have more uncertainty than integrated systems
    poseEstimator = new GenericPoseEstimator(
        new Pose2d(),
        GenericPoseEstimator.createVector(0.12, 0.12, 0.08),  // Higher uncertainty
        GenericPoseEstimator.createVector(0.20, 0.20, 0.20)   // Trust vision more
    );
}

@Override
public void loop() {
    // Update custom odometry
    double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    customOdometry.update(heading);
    
    // Update pose estimator
    Pose2d estimatedPose = poseEstimator.update(customOdometry.getRobotPose());
    
    // Handle vision...
}
```

### Example 4: Dynamic Vision Trust Based on Conditions

```java
public void handleAdvancedVisionMeasurements() {
    if (visionSystem.hasNewMeasurement()) {
        VisionMeasurement measurement = visionSystem.getLatestMeasurement();
        
        // Method 1: Distance-based standard deviations
        SimpleMatrix distanceStdDevs = visionStdDevCalc.calculateStdDevsFromDistance(
            measurement.getDistanceToTag(),
            measurement.getTagSize(),
            measurement.getViewingAngle(),
            measurement.getNumberOfTags()
        );
        
        // Method 2: Confidence-based (if your vision system provides confidence)
        SimpleMatrix confidenceStdDevs = visionStdDevCalc.calculateStdDevsFromConfidence(
            measurement.getConfidenceLevel()
        );
        
        // Method 3: Odometry drift compensation
        double timeSinceLastVision = getCurrentTime() - lastVisionTime;
        double distanceTraveled = calculateDistanceTraveled();
        SimpleMatrix driftStdDevs = visionStdDevCalc.calculateStdDevsFromOdometryDrift(
            timeSinceLastVision, distanceTraveled
        );
        
        // Combine methods (use the most conservative - highest std dev)
        SimpleMatrix finalStdDevs = new SimpleMatrix(3, 1);
        for (int i = 0; i < 3; i++) {
            double maxStdDev = Math.max(
                Math.max(distanceStdDevs.get(i, 0), confidenceStdDevs.get(i, 0)),
                driftStdDevs.get(i, 0)
            );
            finalStdDevs.set(i, 0, maxStdDev);
        }
        
        poseEstimator.addVisionMeasurement(
            measurement.getRobotPose(),
            measurement.getTimestamp(),
            finalStdDevs
        );
        
        lastVisionTime = getCurrentTime();
    }
}
```

## Configuration Guidelines

### Odometry Standard Deviations
- **High-precision systems** (Pinpoint, professional odometry): 0.02 - 0.08 meters/radians
- **Good custom systems** (three-wheel, well-tuned): 0.05 - 0.12 meters/radians  
- **Basic systems** (two-wheel, less precise): 0.10 - 0.20 meters/radians

### Vision Standard Deviations
- **Close-range, good lighting** (< 1 meter): 0.05 - 0.15 meters/radians
- **Medium range** (1-3 meters): 0.15 - 0.30 meters/radians
- **Long range** (> 3 meters): 0.30 - 0.60 meters/radians

### Key Benefits of This Approach

1. **Drive-Type Independent**: Works with swerve, mecanum, differential, holonomic drives
2. **Modular Design**: Easy to swap odometry systems without changing estimator code
3. **Advanced Error Modeling**: Sophisticated handling of measurement uncertainties
4. **Latency Compensation**: Handles vision measurement delays properly
5. **Real-World Factors**: Accounts for distance, viewing angles, lighting, etc.

This design gives you the flexibility to use any odometry system while maintaining the statistical rigor of the WPILib approach, but adapted for FTC's typical hardware setups.