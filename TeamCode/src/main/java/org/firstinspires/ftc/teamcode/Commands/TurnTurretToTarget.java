package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.Utils;

/** A command to follow a path generated from input parameters
 * */
public class TurnTurretToTarget extends CommandBase {

    // our current facing angle (degrees)
    double turretCurrentAngle;

    // out tturret's target angle (degrees)
    double turretTargetAngle;
    double turretRemainingError;

    // translation of target angle given robot heading now
    double targetTranslationAngle;

    // set starting robot gyro direction as turret will be aligned to this as 0 degrees
    public double startingGyroDegrees = RobotContainer.gyro.getYawAngle();

    /** Turn to/by angle
     * Input: angle - degrees (-180<angle<180)
     * relative - true if relative to current angle, false if absolute to field
     * clockwise - true if rotate clockwise, false if counter-clockwise */
    public TurnTurretToTarget() {

        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(RobotContainer.turret);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

//        // determine present angle of turret
//        turretCurrentAngle = RobotContainer.turret.getTurretDegrees();
//
//        // get our current position and the target position
//        Pose2d pose = RobotContainer.odometry.getCurrentPos();
//        Translation2d targetPose = RobotContainer.targeting.GetShotTaget();
//
//        // determine target angle from Robot field pose
//        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
//        turretTargetAngle = Math.toDegrees(angle_rad) - 180.0; // switched as this turret shoots forward not back
//
//        turretRemainingError = Utils.AngleDifference(turretTargetAngle, turretCurrentAngle);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
//        turretCurrentAngle = RobotContainer.turret.getTurretDegrees();
//
//        // determine remaining angle to turn turret
//        turretRemainingError = Utils.AngleDifference(turretTargetAngle, turretCurrentAngle);
//
//        // get our current position and the target position
//        Pose2d pose = RobotContainer.odometry.getCurrentPos();
//        Translation2d targetPose = RobotContainer.targeting.GetShotTaget();
//
//        // determine target angle from Robot field pose
//        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
//        turretTargetAngle = Math.toDegrees(angle_rad) - 180.0; // switched as this by 180 degrees as turret shoots forward not back
//
//        // this is the right idea but maybe the wrong way...
//        // consider the robot angle (where -90 is "home" for Blue alliance) and translate the
//        // turretTargetAngle into a robot relative turretTargetAngle
//        turretTargetAngle -= Utils.AngleDifference(startingGyroDegrees, RobotContainer.gyro.getYawAngle());
//
//        // rotate turret until it's within 0.5 degrees
//        //if (turretRemainingError > 0.5) {
//        RobotContainer.turret.moveTurret(turretTargetAngle, turretRemainingError);
//        //}
//
//        // RobotContainer.Panels.FTCTelemetry.addData("TargetAngle", m_endangle);
//        // RobotContainer.Panels.FTCTelemetry.addData("AngleError", Math.max(-10.0, Math.min(10.0, m_angleerror)));
//        // RobotContainer.Panels.FTCTelemetry.update();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // stop robot
        //RobotContainer.turret.moveTurret(turretCurrentAngle, 0.0);
    }

}