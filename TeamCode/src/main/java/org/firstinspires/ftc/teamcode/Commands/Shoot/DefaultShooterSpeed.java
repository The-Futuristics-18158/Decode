package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class DefaultShooterSpeed extends CommandBase {

    // constructor
    public DefaultShooterSpeed() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.shooter);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
//        // only turn on shooter once we have at least one artifact
//        if (RobotContainer.colour.isLeftArtifactPresent() || RobotContainer.colour.isRightArtifactPresent()){
//            //RobotContainer.shooter.SetFlywheelSpeed(1000);
//            RobotContainer.shooter.SetFlywheelSpeed(Math.max(0.0,RobotContainer.targeting.IdleSpeed()));
//        } else{
//            RobotContainer.shooter.SetFlywheelSpeed(0);
//        }

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}