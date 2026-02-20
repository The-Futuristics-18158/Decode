package org.firstinspires.ftc.teamcode.Commands.Intake.JogBack;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class JogBackIntake extends CommandBase {

    // constructor
    public JogBackIntake() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.intake);

    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        RobotContainer.intake.intakeReverse();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return true;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.intake.intakeStop();
    }

}