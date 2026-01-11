package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootSingleGreen;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootSinglePurple;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootAllAnyColor;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootAllObeliskColor;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleRightUptake;
import org.firstinspires.ftc.teamcode.Commands.ClimbCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.ManualDrive;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Drive.TurnTo;
import org.firstinspires.ftc.teamcode.Commands.Intake.HuntModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoot.DefaultShooterSpeed;
import org.firstinspires.ftc.teamcode.Subsystems.ArtifactCamera;
import org.firstinspires.ftc.teamcode.Subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.Subsystems.Climb.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.HoodTiltSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OperatorControlsSubsystem;
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
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

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
    public static OperatorControlsSubsystem operatorControls;
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
    public static HoodTiltSubsystem hoodtilt;
    public static Obelisk obelisk;
    public static GoalTargeting targeting;
    public static ShotBlockServo shotblock;
    public static ClimbSubsystem climb;
    public static Blinkin blinkin;
    public static ArtifactCamera artifactCamera;

    // Angle of the robot at the start of auto
    public static double RedStartAngle = 90;
    public static double BlueStartAngle = -90;

    // List of robot control and expansion hubs - used for caching of I/O
    static List<LynxModule> allHubs;

    // Robot Modes
    public enum Modes { Off, AutoInit, Auto, TeleOp}
    private static Modes CurrentRobotMode;

    public static int artifactsInRamp = 0;

    // robot initialization - common to both auto and teleop
    // mode - current opmode that is being run
    // RedAlliance - true if robot in red alliance, false if blue
    public static void Init(CommandOpMode mode, boolean RedAlliance) {

        // save pointer to active OpMode
        ActiveOpMode = mode;

        // set alliance colour
        isRedAlliance = RedAlliance;

        // set robot mode - robot is off until we have initialized
        CurrentRobotMode = Modes.Off;

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
        operatorControls = new OperatorControlsSubsystem();
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
        hoodtilt = new HoodTiltSubsystem();
        obelisk = new Obelisk();
        targeting = new GoalTargeting();
        shotblock = new ShotBlockServo();
        climb = new ClimbSubsystem();
        blinkin = new Blinkin();
        artifactCamera = new ArtifactCamera("CookieCam");
    }

    // Robot initialization for teleop - This runs once at initialization of teleop
    public static void Init_TeleOp() {

        // robot is in teleop mode
        CurrentRobotMode = Modes.TeleOp;

        // set drivetrain default command to manual driving mode
        drivesystem.setDefaultCommand(new ManualDrive());

        // set default shooter speed control
        shooter.setDefaultCommand(new DefaultShooterSpeed());

        uptake.LowerRightUptake();
        uptake.LowerLeftUptake();

        // Controller bindings

        // Reset odometry
        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()-> odometry.setCurrentPos
                (AutoFunctions.redVsBlue(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(-90.0)))))));

        // Shoot Green
        driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(new ShootSingleGreen());

//        // Shoot All
//        driverOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new ShootAllAnyColor());

        driverOp.getGamepadButton(GamepadKeys.Button.B).whenHeld(new CycleRightUptake());

        // Shoot Purple
        driverOp.getGamepadButton(GamepadKeys.Button.X).whenHeld(new ShootSinglePurple());

//        // Shoot According to the obelisk reading
//        driverOp.getGamepadButton(GamepadKeys.Button.Y).whenHeld(new ShootAllObeliskColor());


        driverOp.getGamepadButton(GamepadKeys.Button.Y).whenHeld(new CycleLeftUptake());

        // Hunt Mode
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new HuntModeCommand());

        // Manual Intake
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new IntakeCommand());

        // Climb Positioning
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(new MoveToPose(1, 1, AutoFunctions.redVsBlue( new Pose2d( new Translation2d(0.935, 0.81), new Rotation2d(0)))));

        // Climb in three seconds
        driverOp.getGamepadButton(GamepadKeys.Button.START).whenHeld(new ClimbCommand());

//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(()->  operatorControls.increaseRampTotal()));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(()->  operatorControls.decreaseRampTotal()));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(()-> operatorControls.resetRampTotal()));

        // bind commands to buttons
        // bind gyro reset to back button.
        // Note: since reset is very simple command, we can just use 'InstandCommand'
        // instead of creating a full command, just to run one line of java code.

        // example turn to command
        // driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld((new TurnTo(AutoFunctions.redVsBlue(0.0), true, 3.0)));

        // example sequential command
        // driverOp.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new ExampleCommandGroup());


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

        artifactsInRamp = 0;
    }

    // Robot initialization for auto - This runs once at initialization of auto
    public static void Init_Auto() {

        // robot is in auto init mode
        CurrentRobotMode = Modes.AutoInit;

        // perform any autonomous-specific initialization here
        // set limelight to obelisk pipeline
        if (RobotContainer.isRedAlliance == false){
            limeLight.SetPipelineMode(1);
        }else{
            limeLight.SetPipelineMode(2);
        }

        //obelisk.StartObeliskScan();
    }

    // Robot starting code for auto - This runs once at start of auto
    public static void Start_Auto() {

        // robot is in auto mode
        CurrentRobotMode = Modes.Auto;

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

            // show obelisk status - only if in auto-init mode
            if (GetCurrentMode()==Modes.AutoInit) {
                if (limeLight.getObeliskID()== LimeLight.tagId.TAG_GPP)
                     RCTelemetry.addLine("See GPP Tag");
                 else if (limeLight.getObeliskID()== LimeLight.tagId.TAG_PGP)
                     RCTelemetry.addLine("See PGP Tag");
                 else if (limeLight.getObeliskID()== LimeLight.tagId.TAG_PPG)
                     RCTelemetry.addLine("See PPG Tag");
                 else
                     RCTelemetry.addLine("No Obelisk Tag");
            }


            RCTelemetry.update();
        }
    }

    public static boolean isRedAlliance() {
        return isRedAlliance;
    }

    // Returns the current robot mode
    public static Modes GetCurrentMode() { return CurrentRobotMode; }

    public static double getBlueStartAngle() {
        return BlueStartAngle;
    }

    public static double getRedStartAngle() {
        return RedStartAngle;
    }

}
