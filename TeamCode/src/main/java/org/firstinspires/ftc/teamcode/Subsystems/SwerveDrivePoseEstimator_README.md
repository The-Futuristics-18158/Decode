# SwerveDrivePoseEstimator for FTC

This is a port of WPILib's SwerveDrivePoseEstimator to FTC's FTCLib, allowing FTC teams to use advanced swerve drive pose estimation with vision fusion.

## Overview

The SwerveDrivePoseEstimator combines swerve drive encoder odometry with vision measurements (like AprilTags) to provide highly accurate robot pose estimation. It uses Kalman filtering to optimally fuse multiple sensor inputs while accounting for measurement noise and latency.

## Key Features

- **Sensor Fusion**: Combines wheel encoders, gyroscope, and vision measurements
- **Latency Compensation**: Handles delayed vision measurements properly
- **Noise Handling**: Accounts for measurement noise in all sensors
- **Drop-in Replacement**: Can replace basic odometry with minimal code changes
- **Configurable Trust**: Adjust how much to trust different measurements

## Core Classes

### 1. SwerveModulePosition
Represents the position of a swerve module (distance traveled and current angle).

```java
SwerveModulePosition position = new SwerveModulePosition(
    driveDistanceMeters, 
    new Rotation2d(moduleAngleRadians)
);
```

### 2. SwerveModuleState  
Represents the state of a swerve module (velocity and angle).

```java
SwerveModuleState state = new SwerveModuleState(
    speedMetersPerSecond,
    new Rotation2d(targetAngleRadians)
);

// Optimize to minimize rotation
SwerveModuleState optimized = SwerveModuleState.optimize(state, currentAngle);
```

### 3. SwerveDriveKinematics
Converts between chassis speeds and individual module states.

```java
// Define module locations relative to robot center
SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(0.3, 0.3),   // Front Left
    new Translation2d(0.3, -0.3),  // Front Right  
    new Translation2d(-0.3, 0.3),  // Back Left
    new Translation2d(-0.3, -0.3)  // Back Right
);

// Convert chassis speeds to module states
ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1.0, 0.5, 0.2);
SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

// Convert module states back to chassis speeds
ChassisSpeeds speeds = kinematics.toChassisSpeeds(moduleStates);
```

### 4. SwerveDriveOdometry
Basic odometry using only wheel encoders and gyroscope.

```java
SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics,
    gyroAngle,
    modulePositions,
    initialPose
);

// Update with new measurements
Pose2d currentPose = odometry.update(newGyroAngle, newModulePositions);
```

### 5. SwerveDrivePoseEstimator
Advanced pose estimation with vision fusion (the main class you'll use).

```java
SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics,
    initialGyroAngle,
    initialModulePositions,
    initialPose
);

// Update every loop with encoder/gyro data
Pose2d estimatedPose = poseEstimator.update(gyroAngle, modulePositions);

// Add vision measurements when available
poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
```

## Integration Example

Here's how to integrate the pose estimator into your swerve drive subsystem:

```java
public class SwerveDriveSubsystem extends SubsystemBase {
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModulePosition[] modulePositions;
    
    public SwerveDriveSubsystem() {
        // Initialize kinematics with your module positions
        kinematics = new SwerveDriveKinematics(/* module translations */);
        
        // Initialize module position tracking
        modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition();
        }
        
        // Create pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroAngle(),
            modulePositions,
            new Pose2d() // Starting pose
        );
    }
    
    @Override
    public void periodic() {
        // Update module positions from your encoders
        updateModulePositions();
        
        // Update pose estimator every loop
        poseEstimator.update(getGyroAngle(), modulePositions);
    }
    
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
```

## Vision Integration

### Basic Vision Integration
```java
// When you get AprilTag data
if (aprilTagDetected) {
    Pose2d robotPose = calculateRobotPoseFromTag(tagData);
    double timestamp = tagData.getTimestamp();
    
    poseEstimator.addVisionMeasurement(robotPose, timestamp);
}
```

### Advanced Vision Integration with Distance-Based Trust
```java
public void processAprilTagMeasurement(AprilTagData tagData) {
    Pose2d robotPose = calculateRobotPoseFromTag(tagData);
    double distance = robotPose.getTranslation().getDistance(getCurrentPose().getTranslation());
    
    // Adjust trust based on distance - farther = less trust
    double xyStdDev = Math.max(0.1, 0.01 * distance);
    double rotStdDev = Math.max(0.05, 0.005 * distance);
    
    SimpleMatrix visionStdDevs = SwerveDrivePoseEstimator.createMatrix(
        SwerveDrivePoseEstimator.VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
    );
    
    poseEstimator.addVisionMeasurement(robotPose, tagData.getTimestamp(), visionStdDevs);
}
```

## Tuning Parameters

### State Standard Deviations
These control how much you trust your odometry:
- **Lower values** = trust odometry more, slower to incorporate vision corrections
- **Higher values** = trust odometry less, faster to incorporate vision corrections

```java
// Conservative (trust odometry)
SimpleMatrix stateStdDevs = SwerveDrivePoseEstimator.createMatrix(
    SwerveDrivePoseEstimator.VecBuilder.fill(0.05, 0.05, 0.05)
);

// Aggressive (less trust in odometry)  
SimpleMatrix stateStdDevs = SwerveDrivePoseEstimator.createMatrix(
    SwerveDrivePoseEstimator.VecBuilder.fill(0.2, 0.2, 0.2)
);
```

### Vision Standard Deviations
These control how much you trust vision measurements:
- **Lower values** = trust vision more, apply larger corrections
- **Higher values** = trust vision less, apply smaller corrections

```java
// High trust in vision (good lighting, close to tags)
SimpleMatrix visionStdDevs = SwerveDrivePoseEstimator.createMatrix(
    SwerveDrivePoseEstimator.VecBuilder.fill(0.1, 0.1, 0.1)
);

// Low trust in vision (poor conditions, far from tags)
SimpleMatrix visionStdDevs = SwerveDrivePoseEstimator.createMatrix(
    SwerveDrivePoseEstimator.VecBuilder.fill(1.0, 1.0, 1.0)
);
```

## Best Practices

### 1. Reject Bad Vision Data
```java
public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    Pose2d currentPose = getPose();
    double distance = visionPose.getTranslation().getDistance(currentPose.getTranslation());
    
    // Reject measurements that are too far from current estimate
    if (distance > 2.0) { // 2 meters
        return;
    }
    
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
}
```

### 2. Dynamic Standard Deviations
```java
public void addAprilTagMeasurement(AprilTagData data) {
    double distance = data.getDistance();
    int numTags = data.getNumTagsSeen();
    
    // Better measurements with more tags and closer distance
    double baseStdDev = 0.1;
    double distanceFactor = Math.max(1.0, distance / 3.0);
    double tagFactor = Math.max(1.0, 4.0 / numTags);
    
    double stdDev = baseStdDev * distanceFactor * tagFactor;
    
    SimpleMatrix visionStdDevs = SwerveDrivePoseEstimator.createMatrix(
        SwerveDrivePoseEstimator.VecBuilder.fill(stdDev, stdDev, stdDev * 2)
    );
    
    poseEstimator.addVisionMeasurement(data.getPose(), data.getTimestamp(), visionStdDevs);
}
```

### 3. Reset When Necessary
```java
// Reset pose when you know the exact position (like at start of autonomous)
public void resetPose(Pose2d knownPose) {
    poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), knownPose);
}
```

## Troubleshooting

### Poor Pose Accuracy
1. **Check encoder scaling** - Ensure distance calculations are correct
2. **Verify gyro** - Make sure gyro angles are in correct units (radians)
3. **Tune standard deviations** - Adjust trust levels between sensors
4. **Check vision timestamps** - Ensure timestamps are consistent

### Vision Corrections Too Aggressive/Conservative
1. **Adjust vision standard deviations** based on measurement quality
2. **Implement rejection criteria** for bad vision measurements
3. **Use dynamic trust** based on distance, lighting, number of tags

### Pose Drifting
1. **Check module positions** - Ensure they're updated correctly each loop
2. **Verify kinematics** - Module positions must match physical layout
3. **Add more vision measurements** - More frequent corrections help

## Dependencies

This implementation requires:
- FTCLib (`com.arcrobotics.ftclib`)
- EJML (`org.ejml.simple.SimpleMatrix`) - Usually included with FTCLib
- Standard FTC SDK hardware classes

## License

This is a port of WPILib code, which is licensed under the BSD 3-Clause License. This port maintains the same license for compatibility.