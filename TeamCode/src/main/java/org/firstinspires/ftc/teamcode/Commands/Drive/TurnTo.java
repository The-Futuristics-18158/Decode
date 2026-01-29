package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.Trajectory.State;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.constraint.MecanumDriveKinematicsConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

import java.util.List;

/** A command to follow a path generated from input parameters
 * */
public class TurnTo extends CommandBase {

    double m_angle;
    boolean m_relative;
    double m_endangle;
    double m_timeout;

    // speed to rotate robot - determined by PID controller
    double m_rotatespeed;
    double m_angleerror;

    // PID gains for rotating robot towards ball target
    double kp = 0.075;
    double ki = 0.2;
    double kd = 0.0025;
    PIDController pidController = new PIDController(kp, ki, kd);

    // on-target timer
    ElapsedTime OnTargetTime;

    // Timeout timer
    ElapsedTime TimeOutTime;


    /** Turn to/by angle
     * Input: angle - degrees (-180<angle<180)
     * relative - true if relative to current angle, false if absolute to field
     * clockwise - true if rotate clockwise, false if counter-clockwise */
    public TurnTo(double angle, boolean relative, double timeout) {

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.drivesystem);

        m_angle = angle;
        m_relative = relative;
        m_timeout = timeout;

        OnTargetTime = new ElapsedTime();
        TimeOutTime = new ElapsedTime();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // our current facing angle
        double ourcurrentangle = RobotContainer.gyro.getYawAngle();

        // set our ending angle based on whether angle is relative or absolute
        if (m_relative)
            m_endangle = ourcurrentangle + m_angle;
        else
            m_endangle = m_angle;

        // reset PID controller
        pidController.reset();

        m_angleerror = 0.0;

        OnTargetTime.reset();
        TimeOutTime.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // determine speed to rotate robot
        m_angleerror = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());

        // only integrate if within 10deg
        if (Math.abs(m_angleerror) < 10.0)
            pidController.setI(ki);
        else
            pidController.setI(0.0);


        // execute PID controller
        m_rotatespeed = pidController.calculate(m_angleerror);

        // rotate robot
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, m_rotatespeed);

        // if we are within target, allow timer to count up
        if (Math.abs(m_angleerror) >=3.0)
            OnTargetTime.reset();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // stop robot
        RobotContainer.drivesystem.RobotDrive(0.0,0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // we are finished when within tolerance of target for required duration,
        // or have timed out
        return (OnTargetTime.seconds()>0.25 || TimeOutTime.seconds() >=m_timeout);
    }
}