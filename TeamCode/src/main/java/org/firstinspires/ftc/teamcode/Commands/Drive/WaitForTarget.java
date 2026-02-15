package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

/** A command to follow a path generated from input parameters
 * */
public class WaitForTarget extends CommandBase {

    double m_endangle;
    double m_timeout;
    double m_angleerror;

    // on-target timer
    ElapsedTime OnTargetTime;

    // Timeout timer
    ElapsedTime TimeOutTime;


    /** Constructor
     * Input: timeout in s. Command ends after timeout reached, regardless of state of robot */
    public WaitForTarget(double timeout) {

        m_timeout = timeout;
        OnTargetTime = new ElapsedTime();
        TimeOutTime = new ElapsedTime();

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // determine target angle

        // get our current position and the target position
        Pose2d pose = RobotContainer.odometry.getCurrentPos();
        Translation2d targetPose = RobotContainer.targeting.GetShotTaget();

        // determine target angle
        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        m_endangle = Math.toDegrees(angle_rad);

        OnTargetTime.reset();
        TimeOutTime.reset();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // determine speed to rotate robot
        m_angleerror = Utils.AngleDifference(m_endangle, RobotContainer.gyro.getYawAngle());

        // if we are within target, allow timer to count up
        if (Math.abs(m_angleerror) >=1.0)
            OnTargetTime.reset();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // we are finished when within tolerance of target for required duration,
        // or have timed out
        return (OnTargetTime.seconds()>0.25 || TimeOutTime.seconds() >=m_timeout);
    }
}