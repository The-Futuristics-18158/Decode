package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class JogBackIntakeFull extends CommandBase {

    // constructor
    public JogBackIntakeFull() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.intake);

    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        if (RobotContainer.artifactCamera.IsOverloadPresent())
            RobotContainer.intake.intakeReverse();
        else
            RobotContainer.intake.intakeStop();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return !RobotContainer.artifactCamera.IsOverloadPresent();
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.intake.intakeStop();
    }

}