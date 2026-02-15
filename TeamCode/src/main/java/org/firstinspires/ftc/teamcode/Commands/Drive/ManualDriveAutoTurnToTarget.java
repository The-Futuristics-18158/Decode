package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;


// command controls mecanum in manual mode
public class ManualDriveAutoTurnToTarget extends CommandBase {

    double powerFactor;
    double basePowerFacter = 0.65;
    double boostPowerFacter = 0.35;
    double slowPowerFactor = 0.4;
    double m_endangle;

    // speed to rotate robot - determined by PID controller
    double m_angleerror;

    // PID gains for rotating robot towards ball target
    public static double kpmax = 0.11; // was 0.15 Feb 5/2026 KN
    public static double kpmin = 0.04; // was 0.05 Feb 5/2026 KN
    public static double kp_deg = 90.0;
    public static double ki = 0.2; // 0.20;
    public static double ki_range = 5.0;
    public static double ki_zerorange = 0.5;
    public static double kd = 0.019; // 0.0035;
    public static double kd_deg = 110.0;

    // filtered error angle
    double filtered_error;
    public static double LPFvalue = 0.93;

    PIDController pidController = new PIDController(kpmax, ki, kd);


    // constructor
    public ManualDriveAutoTurnToTarget() {

        // this command requires mecanum drive subsystem
        addRequirements(RobotContainer.drivesystem);

    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // determine target angle

        // get our current position and the target position
        Pose2d pose = RobotContainer.odometry.getCurrentPos();
        Translation2d targetPose = RobotContainer.targeting.GetShotTaget();

        // determine target angle
        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        m_endangle = Math.toDegrees(angle_rad);

        // reset PID controller
        pidController.reset();
        m_angleerror = 0.0;

        // determine initial error
        filtered_error = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get joystick input - for competition
        double dX = RobotContainer.ActiveOpMode.gamepad1.left_stick_x;
        double dY = -RobotContainer.ActiveOpMode.gamepad1.left_stick_y;
        double speedTrigger = RobotContainer.ActiveOpMode.gamepad1.right_trigger;
        double slowTrigger = RobotContainer.ActiveOpMode.gamepad1.left_trigger;

        if (RobotContainer.isRedAlliance==false){
            dX = dX * -1;
            dY = dY * -1;
        }

        // implement dead-zoning of joystick inputs
        dX = Math.abs(dX) > 0.05 ? dX : 0;
        dY = Math.abs(dY) > 0.05 ? dY : 0;


        powerFactor = basePowerFacter + (speedTrigger * boostPowerFacter) - (slowTrigger * slowPowerFactor);

        // Since the drive was shifted to closed loop (i.e. requested velocities), change joystick input max values
        // to MAX_SPEED values.
        powerFactor = powerFactor * RobotContainer.drivesystem.MAX_SPEED;

        // include power factor to get full x,y and omega speeds beings requested
        dX *= powerFactor;
        dY *= powerFactor;

        // ---------- Auto Aim to Target - Robot Rotation Control ----------

        // determine target angle

        // get our current position and the target position
        Pose2d pose = RobotContainer.odometry.getCurrentPos();
        Translation2d targetPose = RobotContainer.targeting.GetShotTaget();

        // determine target angle
        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        m_endangle = Math.toDegrees(angle_rad);

        // determine speed to rotate robot
        m_angleerror = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());

        // apply LPF to angle error
        filtered_error = LPFvalue*filtered_error + (1.0-LPFvalue)*m_angleerror;

        // set p gain
        double pgain = kpmax - (kpmax - kpmin) * Math.abs(m_angleerror) / kp_deg;
        pgain = Math.max(kpmin, pgain);
        pgain = Math.min(kpmax, pgain);
        pidController.setP(pgain);

        // only integrate if within 10deg
        if (Math.abs(m_angleerror) < ki_range)
            pidController.setI(ki);
        else
            pidController.setI(0.0);

        // zero out the integrated error if very close to target
        if (Math.abs(m_angleerror) < ki_zerorange)
            pidController.reset();

        double gain = Math.max(0.0, kd*(1.0-Math.abs(filtered_error)/kd_deg));
        pidController.setD(gain);

        // execute PID controller
        double omega  = pidController.calculate(m_angleerror);


        // ---------- Drive Robot ----------

        // drive robot
        RobotContainer.drivesystem.FieldDrive(dX, dY, omega);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        // stop robot
        RobotContainer.drivesystem.RobotDrive(0.0,0.0, 0.0);
    }

}