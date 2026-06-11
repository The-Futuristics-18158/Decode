package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class VaccuumMode extends CommandBase {

    // constructor
    public VaccuumMode() {

        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        // Calls the shotblock getter function then sets the opposite when called.
        if (RobotContainer.shotblock.ShotBlocked()){
            RobotContainer.shotblock.Unblock();
        }else {
            RobotContainer.shotblock.Block();
        }

        // Calls the Ramp getter function then sets the opposite when called.
//        if (RobotContainer.ramplift.RampIsUp()){
//            RobotContainer.ramplift.Lower();
//        }else {
//            RobotContainer.ramplift.Raise();
//        }
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return true;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}