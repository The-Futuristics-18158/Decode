package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class ClimbCommand extends CommandBase {

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
        RobotContainer.drivesystem.RobotDrive(0.0,0.0,0.0);
        RobotContainer.climb.moveClimb();

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