package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;
import org.firstinspires.ftc.teamcode.Utility.Utils;

// command template
public class ClimbCommand extends CommandBase {

    Pose2d boxPos = AutoFunctions.redVsBlue( new Pose2d(new Translation2d( 0.935, 0.81), new Rotation2d(0)));
    double error = 0.20; // only works within 20 cm

    // constructor
    public ClimbCommand() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.climb);
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
//        if (RobotContainer.odometry.getCurrentPos().getX() > (boxPos.getX() - error) &&
//                RobotContainer.odometry.getCurrentPos().getX() < (boxPos.getX() + error) &&
//                RobotContainer.odometry.getCurrentPos().getY() > (boxPos.getY() - error) &&
//                RobotContainer.odometry.getCurrentPos().getY() < (boxPos.getY() + error)) {

            RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0);
            RobotContainer.climb.moveClimb();

      //  }

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.climb.climbStop();
    }

}