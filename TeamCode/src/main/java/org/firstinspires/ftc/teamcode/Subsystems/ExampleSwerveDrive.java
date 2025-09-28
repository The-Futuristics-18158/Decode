package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Example swerve drive subsystem that integrates the ported SwerveDrivePoseEstimator.
 * 
 * This demonstrates how to use the pose estimator with FTC hardware and integrate
 * vision measurements for improved accuracy.
 * 
 * Note: This is a simplified example. Real implementations would need proper
 * swerve module classes, PID controllers, and appropriate hardware configurations.
 */
public class ExampleSwerveDrive extends SubsystemBase {
    
    // Swerve drive physical constants
    private static final double WHEEL_BASE = 0.5842; // meters
    private static final double TRACK_WIDTH = 0.5842; // meters
    private static final double WHEEL_DIAMETER = 0.1016; // 4 inches in meters
    private static final double DRIVE_GEAR_RATIO = 6.75; // MK4i L2 ratio
    private static final double ANGLE_GEAR_RATIO = 12.8; // MK4i angle ratio
    private static final double MAX_SPEED_METERS_PER_SECOND = 4.96;

    // Hardware
    private final DcMotorEx frontLeftDriveMotor;
    private final DcMotorEx frontRightDriveMotor;
    private final DcMotorEx backLeftDriveMotor;
    private final DcMotorEx backRightDriveMotor;
    
    private final DcMotorEx frontLeftTurnMotor;
    private final DcMotorEx frontRightTurnMotor;
    private final DcMotorEx backLeftTurnMotor;
    private final DcMotorEx backRightTurnMotor;
    
    private final AnalogInput frontLeftEncoder;
    private final AnalogInput frontRightEncoder;
    private final AnalogInput backLeftEncoder;
    private final AnalogInput backRightEncoder;

    // Kinematics and pose estimation
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    
    // Module positions for tracking
    private SwerveModulePosition[] modulePositions;
    private SwerveModulePosition[] lastModulePositions;
    
    // Timing
    private ElapsedTime timer = new ElapsedTime();

    public ExampleSwerveDrive() {
        // Initialize hardware
        frontLeftDriveMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDriveMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDriveMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDriveMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "backRightDrive");
        
        frontLeftTurnMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "frontLeftTurn");
        frontRightTurnMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "frontRightTurn");
        backLeftTurnMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "backLeftTurn");
        backRightTurnMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "backRightTurn");
        
        frontLeftEncoder = RobotContainer.ActiveOpMode.hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = RobotContainer.ActiveOpMode.hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = RobotContainer.ActiveOpMode.hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = RobotContainer.ActiveOpMode.hardwareMap.get(AnalogInput.class, "backRightEncoder");

        // Configure motors
        configureMotors();

        // Create kinematics object
        kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),  // Front Left
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // Front Right
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // Back Left
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0) // Back Right
        );

        // Initialize module positions
        modulePositions = new SwerveModulePosition[4];
        lastModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition();
            lastModulePositions[i] = new SwerveModulePosition();
        }

        // Create pose estimator with default standard deviations
        // You can adjust these based on your robot's characteristics
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            RobotContainer.gyro.getYawAngle() != 0 ? new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle())) : new Rotation2d(),
            modulePositions,
            new Pose2d() // Start at origin
        );

        timer.reset();
    }

    private void configureMotors() {
        // Configure drive motors
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure turn motors
        frontLeftTurnMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightTurnMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftTurnMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightTurnMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake mode for precision
        setMotorBrakeMode(true);
    }

    private void setMotorBrakeMode(boolean brake) {
        DcMotorEx.ZeroPowerBehavior behavior = brake ? 
            DcMotorEx.ZeroPowerBehavior.BRAKE : DcMotorEx.ZeroPowerBehavior.FLOAT;

        frontLeftDriveMotor.setZeroPowerBehavior(behavior);
        frontRightDriveMotor.setZeroPowerBehavior(behavior);
        backLeftDriveMotor.setZeroPowerBehavior(behavior);
        backRightDriveMotor.setZeroPowerBehavior(behavior);

        frontLeftTurnMotor.setZeroPowerBehavior(behavior);
        frontRightTurnMotor.setZeroPowerBehavior(behavior);
        backLeftTurnMotor.setZeroPowerBehavior(behavior);
        backRightTurnMotor.setZeroPowerBehavior(behavior);
    }

    @Override
    public void periodic() {
        // Update module positions from encoders
        updateModulePositions();

        // Update pose estimator with current gyro angle and module positions
        Rotation2d gyroAngle = new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle()));
        poseEstimator.update(gyroAngle, modulePositions);

        // Copy current positions to last positions for next iteration
        for (int i = 0; i < 4; i++) {
            lastModulePositions[i] = new SwerveModulePosition(
                modulePositions[i].distanceMeters, 
                modulePositions[i].angle
            );
        }
    }

    /**
     * Drive the robot using field-relative chassis speeds.
     *
     * @param xSpeed Forward/backward speed in m/s (positive = forward)
     * @param ySpeed Left/right speed in m/s (positive = left)
     * @param rotSpeed Rotation speed in rad/s (positive = counterclockwise)
     * @param fieldRelative Whether speeds are field-relative or robot-relative
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds;
        
        if (fieldRelative) {
            // Convert field-relative speeds to robot-relative
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, getPose().getRotation());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        // Convert chassis speeds to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize wheel speeds if any exceed maximum
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_SPEED_METERS_PER_SECOND);

        // Set module states
        setModuleStates(moduleStates);
    }

    /**
     * Set the desired state for each swerve module.
     */
    private void setModuleStates(SwerveModuleState[] states) {
        // This is a simplified example. In a real implementation, you would:
        // 1. Use PID controllers for both drive and turn motors
        // 2. Optimize module angles to minimize rotation
        // 3. Handle encoder wraparound and calibration
        
        for (int i = 0; i < states.length; i++) {
            SwerveModuleState state = states[i];
            
            // Convert speed to motor power (simplified)
            double drivePower = state.speedMetersPerSecond / MAX_SPEED_METERS_PER_SECOND;
            
            // Set drive motor power
            switch (i) {
                case 0: // Front Left
                    frontLeftDriveMotor.setPower(drivePower);
                    break;
                case 1: // Front Right
                    frontRightDriveMotor.setPower(drivePower);
                    break;
                case 2: // Back Left
                    backLeftDriveMotor.setPower(drivePower);
                    break;
                case 3: // Back Right
                    backRightDriveMotor.setPower(drivePower);
                    break;
            }
            
            // Note: Turn motor control would require PID and proper angle feedback
            // This is left as an exercise for the implementer
        }
    }

    /**
     * Update module positions from encoder readings.
     * This is a simplified version - real implementation would need proper calibration.
     */
    private void updateModulePositions() {
        // Front Left (index 0)
        double frontLeftDistance = encoderTicksToMeters(frontLeftDriveMotor.getCurrentPosition());
        Rotation2d frontLeftAngle = new Rotation2d(analogToRadians(frontLeftEncoder.getVoltage()));
        modulePositions[0] = new SwerveModulePosition(frontLeftDistance, frontLeftAngle);

        // Front Right (index 1)
        double frontRightDistance = encoderTicksToMeters(frontRightDriveMotor.getCurrentPosition());
        Rotation2d frontRightAngle = new Rotation2d(analogToRadians(frontRightEncoder.getVoltage()));
        modulePositions[1] = new SwerveModulePosition(frontRightDistance, frontRightAngle);

        // Back Left (index 2)
        double backLeftDistance = encoderTicksToMeters(backLeftDriveMotor.getCurrentPosition());
        Rotation2d backLeftAngle = new Rotation2d(analogToRadians(backLeftEncoder.getVoltage()));
        modulePositions[2] = new SwerveModulePosition(backLeftDistance, backLeftAngle);

        // Back Right (index 3)
        double backRightDistance = encoderTicksToMeters(backRightDriveMotor.getCurrentPosition());
        Rotation2d backRightAngle = new Rotation2d(analogToRadians(backRightEncoder.getVoltage()));
        modulePositions[3] = new SwerveModulePosition(backRightDistance, backRightAngle);
    }

    /**
     * Convert encoder ticks to distance in meters.
     */
    private double encoderTicksToMeters(int ticks) {
        // Assumes standard FTC motor encoder (e.g., 28 ticks per revolution for Core Hex Motor)
        double ticksPerRevolution = 28.0 * DRIVE_GEAR_RATIO;
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;
        return (ticks / ticksPerRevolution) * wheelCircumference;
    }

    /**
     * Convert analog encoder voltage to radians.
     * Assumes 0-3.3V maps to 0-2π radians.
     */
    private double analogToRadians(double voltage) {
        return (voltage / 3.3) * 2 * Math.PI;
    }

    /**
     * Get the current estimated robot pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the robot pose to a known position.
     */
    public void resetPose(Pose2d pose) {
        Rotation2d gyroAngle = new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle()));
        poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
    }

    /**
     * Add a vision measurement to improve pose accuracy.
     * 
     * @param visionPose The pose measured by vision (e.g., from AprilTags)
     * @param timestamp The timestamp when the measurement was taken
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    /**
     * Add a vision measurement with custom standard deviations.
     * Use this when you want to adjust trust in the vision measurement
     * based on distance, number of tags seen, etc.
     * 
     * @param visionPose The pose measured by vision
     * @param timestamp The timestamp when the measurement was taken  
     * @param stdDevs Standard deviations [x, y, rotation] in meters and radians
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, double[] stdDevs) {
        poseEstimator.addVisionMeasurement(
            visionPose, 
            timestamp, 
            SwerveDrivePoseEstimator.createMatrix(stdDevs)
        );
    }

    /**
     * Example method showing how to integrate AprilTag vision data.
     * This would be called when new AprilTag data is available.
     */
    public void updateWithAprilTagData(Pose2d tagPose, double timestamp, double distance) {
        // Adjust standard deviations based on distance to tag
        // Closer = more accurate = smaller standard deviations
        double xyStdDev = Math.max(0.1, 0.01 * distance); // Minimum 10cm, increases with distance
        double rotStdDev = Math.max(0.05, 0.005 * distance); // Minimum ~3 degrees
        
        double[] stdDevs = {xyStdDev, xyStdDev, rotStdDev};
        addVisionMeasurement(tagPose, timestamp, stdDevs);
    }

    /**
     * Stop all swerve modules.
     */
    public void stop() {
        frontLeftDriveMotor.setPower(0);
        frontRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);

        frontLeftTurnMotor.setPower(0);
        frontRightTurnMotor.setPower(0);
        backLeftTurnMotor.setPower(0);
        backRightTurnMotor.setPower(0);
    }

    /**
     * Get the current chassis speeds.
     */
    public ChassisSpeeds getChassisSpeeds() {
        // Calculate module states from current positions
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // Calculate velocity from position change (simplified)
            double deltaDistance = modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters;
            double deltaTime = timer.seconds();
            if (deltaTime > 0) {
                double velocity = deltaDistance / deltaTime;
                states[i] = new SwerveModuleState(velocity, modulePositions[i].angle);
            } else {
                states[i] = new SwerveModuleState(0, modulePositions[i].angle);
            }
        }
        
        return kinematics.toChassisSpeeds(states);
    }

    /**
     * Get the swerve drive kinematics object.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
}