package org.firstinspires.ftc.teamcode.Commands.Intake.EndWhenFull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * This command ends only when the robot contains three artifacts in it
 *
 * @author knutt5
 */
public class EndWhenFull3Artifacts extends CommandBase {

    // constructor
    public EndWhenFull3Artifacts() {

        // Note: this command does not require any subsystems
        //addRequirements();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return RobotContainer.colour.isLeftArtifactPresent() &&
                RobotContainer.colour.isRightArtifactPresent() &&
                RobotContainer.distance.isRampArtifactPresent();
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}