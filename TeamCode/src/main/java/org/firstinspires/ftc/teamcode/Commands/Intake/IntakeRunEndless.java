package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;


// This command runs the intake.  This command does not end (runs forever).
// Command intended to be used in a racegroup with other commands.
public class IntakeRunEndless extends CommandBase {

    // constructor
    public IntakeRunEndless() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.intake);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        RobotContainer.intake.intakeSlowRun();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.intakeStop();
    }

}