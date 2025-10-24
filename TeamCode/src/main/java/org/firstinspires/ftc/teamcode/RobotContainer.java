package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Drive.ManualDrive;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.DriveWheelOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.OctQuad;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Utility.TestSlip;
import org.firstinspires.ftc.teamcode.Utility.WriteToFile;

import java.util.List;


public class RobotContainer {

    // active OpMode - used so any subsystem and command and access it and its members
    public static CommandOpMode ActiveOpMode;

    // team alliance color = false if robot on blue alliance, true for red
    public static boolean isRedAlliance;

    // FTC dashboard and telemetries
    public static FtcDashboard DashBoard;
    public static Telemetry DBTelemetry;
    public static Telemetry RCTelemetry;

    // timer used to determine how often to run scheduler periodic
    private static ElapsedTime timer;
    private static ElapsedTime exectimer;
    public static ElapsedTime slipTimer;



    // create robot GamePads
    public static GamepadEx driverOp;
    public static GamepadEx toolOp;

    // create pointers to robot subsystems
    public static Gyro gyro;
    public static OctQuad odometryPod;
    public static DriveTrain drivesystem;
    public static Odometry odometry;
    public static DriveWheelOdometry driveWheelOdometry;
    public static double highestSlip = 0;
    public static double totalSlip = 0;
    public static double xSlip = 0;
    public static double ySlip = 0;
    public static Twist2d xyacc;
    public static double acc = 0;
    public static String slipData;


    // Angle of the robot at the start of auto
    public static double RedStartAngle = 90;
    public static double BlueStartAngle = -90;

    // List of robot control and expansion hubs - used for caching of I/O
    static List<LynxModule> allHubs;

    // Robot initialization for teleop - Run this once at start of teleop
    // mode - current opmode that is being run
    // RedAlliance - true if robot in red alliance, false if blue
    public static void Init_TeleOp(CommandOpMode mode, boolean RedAlliance) {
        // set alliance colour
        isRedAlliance = RedAlliance;

        // Initialize robot subsystems
        Init(mode);

        // set drivetrain default command to manual driving mode
        drivesystem.setDefaultCommand(new ManualDrive());

        // bind commands to buttons
        // bind gyro reset to back button.
        // Note: since reset is very simple command, we can just use 'InstandCommand'
        // instead of creating a full command, just to run one line of java code.
        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenHeld(new InstantCommand(()-> odometry.setCurrentPos(
                new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0)))))
        );
        //driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()-> gyro.resetYawAngle(), gyro));


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

    }


    // Robot initialization for auto - Run this once at start of auto
    // mode - current opmode that is being run
    // RedAlliance - true if robot in red alliance, false if blue
    public static void Init_Auto(CommandOpMode mode, boolean RedAlliance) {
        // set alliance colour
        isRedAlliance = RedAlliance;

        // Initialize robot subsystems
        Init(mode);

        // perform any autonomous-specific initialization here
    }

    // robot initialization - common to both auto and teleop
    private static void Init(CommandOpMode mode) {

        // save pointer to active OpMode
        ActiveOpMode = mode;

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
        exectimer.reset();
        slipTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        slipTimer.reset();


        // set up dashboard and various telemetries
        DashBoard = FtcDashboard.getInstance();
        DBTelemetry = DashBoard.getTelemetry();
        RCTelemetry = ActiveOpMode.telemetry;

        // cancel any commands previously running by scheduler
        CommandScheduler.getInstance().cancelAll();

        // create gamepads
        driverOp = new GamepadEx(ActiveOpMode.gamepad1);
        toolOp = new GamepadEx(ActiveOpMode.gamepad2);

        // create systems
        gyro = new Gyro();
        odometryPod = new OctQuad();
        odometry = new Odometry();
        drivesystem = new DriveTrain();
        driveWheelOdometry = new DriveWheelOdometry();

        highestSlip = 0;
        totalSlip = 0;
        xSlip = 0;
        ySlip = 0;
        TestSlip.linesWritten = 0;
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
            //RCTelemetry.addData("Yaw", gyro.getYawAngle());

            RCTelemetry.addData("Accumulated X slip", TestSlip.getAccumulatedSlip(false).getX());
            RCTelemetry.addData("Accumulated Y slip", TestSlip.getAccumulatedSlip(false).getY());
            RCTelemetry.addData("Accumulated total slip", TestSlip.getAccumulatedSlip(false).getX() + TestSlip.getAccumulatedSlip(false).getY());

            //slipData = WriteToFile.readFile(TestSlip.FILE_NAME);

            Twist2d slip = TestSlip.getSlip();
            totalSlip = Math.abs(slip.dx) + Math.abs(slip.dy);
            if (((Math.abs(slip.dx) + Math.abs(slip.dy))) > highestSlip){
                highestSlip = Math.abs(slip.dx) + Math.abs(slip.dy);
            }
            xSlip = slip.dx;
            ySlip = slip.dy;
            xyacc = TestSlip.getAccel();
            acc = Math.abs(xyacc.dx) + Math.abs(xyacc.dy);

            RCTelemetry.addData("X slip", xSlip);
            RCTelemetry.addData("Y slip", ySlip);
            RCTelemetry.addData("Total slip", totalSlip);
            RCTelemetry.addData("slip", slip.toString());


            RCTelemetry.addData("highest slip", highestSlip);

            TestSlip.writeSlip();

            DBTelemetry.addData("Accumulated X slip", Math.abs(TestSlip.getAccumulatedSlip(false).getX()));
            DBTelemetry.addData("Accumulated Y slip", Math.abs(TestSlip.getAccumulatedSlip(false).getY()));
            DBTelemetry.addData("Accumulated total slip",Math.abs( TestSlip.getAccumulatedSlip(false).getX() )+ Math.abs( TestSlip.getAccumulatedSlip(false).getY()));

            DBTelemetry.addData("acc", acc);
            DBTelemetry.addData("X slip", slip.dx);
            DBTelemetry.addData("Y slip", slip.dy);
            DBTelemetry.addData("Total slip", Math.abs(slip.dx) + Math.abs(slip.dy));
            DBTelemetry.addData("slip", slip.toString());
            DBTelemetry.addData("highest slip", highestSlip);
            DBTelemetry.update();

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
