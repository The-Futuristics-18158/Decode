package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups.GoalSideNineArtifactAuto;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootSingleGreen;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootSinglePurple;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootAllAnyColor;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootAllObeliskColor;
import org.firstinspires.ftc.teamcode.Commands.Drive.ManualDrive;
import org.firstinspires.ftc.teamcode.Commands.Intake.HuntModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoot.DefaultShooterSpeed;
import org.firstinspires.ftc.teamcode.Subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.Subsystems.Climb.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.ColourSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.GoalTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.Obelisk;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Panels;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.PinpointOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.RampCamera;
import org.firstinspires.ftc.teamcode.Subsystems.ShotBlockServo;
import org.firstinspires.ftc.teamcode.Subsystems.UptakeSubsystem;
import java.util.List;


public class RobotContainer {

    // active OpMode - used so any subsystem and command and access it and its members
    public static CommandOpMode ActiveOpMode;

    // team alliance color = false if robot on blue alliance, true for red
    public static boolean isRedAlliance;

    // FTC dashboard and telemetries
    public static Panels Panels;
    public static Telemetry RCTelemetry;

    // timer used to determine how often to run scheduler periodic
    private static ElapsedTime timer;
    private static ElapsedTime exectimer;

    // create robot GamePads
    public static GamepadEx driverOp;
    public static GamepadEx toolOp;

    // create pointers to robot subsystems
    public static Gyro gyro;
    public static PinpointOdometry odometryPod;
    public static DriveTrain drivesystem;
    public static LimeLight limeLight;
    public static RampCamera rampCamera;
    public static Odometry odometry;
    public static ColourSensor colour;
    public static IntakeSubsystem intake;
    public static FlywheelSubsystem shooter;
    public static UptakeSubsystem uptake;
    public static Obelisk obelisk;
    public static GoalTargeting targeting;
    public static ShotBlockServo shotblock;
    public static ClimbSubsystem climb;
    public static Blinkin blinkin;

    // Angle of the robot at the start of auto
    public static double RedStartAngle = 90;
    public static double BlueStartAngle = -90;

    // List of robot control and expansion hubs - used for caching of I/O
    static List<LynxModule> allHubs;


    // robot initialization - common to both auto and teleop
    // mode - current opmode that is being run
    // RedAlliance - true if robot in red alliance, false if blue
    public static void Init(CommandOpMode mode, boolean RedAlliance) {

        // save pointer to active OpMode
        ActiveOpMode = mode;

        // set alliance colour
        isRedAlliance = RedAlliance;

        // create list of robot control and expansion hubs
        // set each for manual caching - cache updated in periodic()
        allHubs = ActiveOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // create and reset timer
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        exectimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();

        // set up dashboard and various telemetries
        Panels = new Panels();
        RCTelemetry = ActiveOpMode.telemetry;

        // cancel any commands previously running by scheduler
        CommandScheduler.getInstance().cancelAll();

        // create gamepads
        driverOp = new GamepadEx(ActiveOpMode.gamepad1);
        toolOp = new GamepadEx(ActiveOpMode.gamepad2);

        // create systems
        gyro = new Gyro();
        odometryPod = new PinpointOdometry();
        odometry = new Odometry();
        drivesystem = new DriveTrain();
        limeLight = new LimeLight();
        rampCamera = new RampCamera("RampCam");
        colour = new ColourSensor();
        intake = new IntakeSubsystem();
        shooter = new FlywheelSubsystem();
        uptake = new UptakeSubsystem();
        obelisk = new Obelisk();
        targeting = new GoalTargeting();
        shotblock = new ShotBlockServo();
        climb = new ClimbSubsystem();
        blinkin = new Blinkin();
    }

    // Robot initialization for teleop - This runs once at initialization of teleop
    public static void Init_TeleOp() {

        // set drivetrain default command to manual driving mode
        drivesystem.setDefaultCommand(new ManualDrive());

        // set default shooter speed control
        shooter.setDefaultCommand(new DefaultShooterSpeed());

        // bind commands to buttons
        // bind gyro reset to back button.
        // Note: since reset is very simple command, we can just use 'InstandCommand'
        // instead of creating a full command, just to run one line of java code.
        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()-> odometry.setCurrentPos(
                new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(-90.0)))))
        );

        driverOp.getGamepadButton(GamepadKeys.Button.START).whenHeld(new GoalSideNineArtifactAuto());

        driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(new ShootSingleGreen());

        driverOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new ShootAllAnyColor());

        driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new ShootSinglePurple());

        driverOp.getGamepadButton(GamepadKeys.Button.Y).whenHeld(new ShootAllObeliskColor());

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new HuntModeCommand());

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new IntakeCommand());

        // example sequential command
        //driverOp.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new ExampleCommandGroup());

        // example of binding more complex command to a button. This would be in a separate command file
        // driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new ExampleCommand());

        // add other button commands here
        // Note: can trigger commands on
        // whenPressed - once when button is pressed
        // whenHeld - runs command while button held, but does not restart if command ends
        // whileHeld - runs command while button held, but will restart command if it ends
        // whenReleased - runs once when button is released
        // togglewhenPressed - turns command on and off at each button press

        // set limelight to apriltag pipeline
        limeLight.SetPipelineMode(0);

    }

    // Robot initialization for auto - This runs once at initialization of auto
    public static void Init_Auto() {

        // perform any autonomous-specific initialization here
        // set limelight to obelisk pipeline
        limeLight.SetPipelineMode(1);

        obelisk.StartObeliskScan();
    }

    // Robot starting code for auto - This runs once at start of auto
    public static void Start_Auto() {

        // perform any autonomous-specific start functions here
        obelisk.RecordPattern();

        // set limelight to apriltag pipeline
        limeLight.SetPipelineMode(0);
    }


    // call this function periodically to operate scheduler
    public static void Periodic() {

        // clear I/O cache for robot control and expansion hubs
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // actual interval time
        double intervaltime = timer.milliseconds();

        // execute robot periodic function 50 times per second (=50Hz)
        if (intervaltime>=20.0) {

            // reset timer
            timer.reset();

            // start execution timer
            exectimer.reset();

            // run scheduler
            CommandScheduler.getInstance().run();

            // report robot odometry on robot controller
            Pose2d position = odometry.getCurrentPos();
            RCTelemetry.addData("fieldX", position.getX());
            RCTelemetry.addData("fieldY", position.getY());
            RCTelemetry.addData("Yaw", position.getRotation().getDegrees());

            // report time interval on robot controller
            RCTelemetry.addData("interval time(ms)", intervaltime);
            RCTelemetry.addData("execute time(ms)", exectimer.milliseconds());
            RCTelemetry.update();
        }
    }

    public static boolean isRedAlliance() {
        return isRedAlliance;
    }

    public static double getBlueStartAngle() {
        return BlueStartAngle;
    }

    public static double getRedStartAngle() {
        return RedStartAngle;
    }

}
