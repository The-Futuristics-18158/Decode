package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;


// This command stops flywheel if no balls detected
public class FlywheelSpeedControl extends CommandBase {


    double x, y;
    // constructor
    public FlywheelSpeedControl() {
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.odometry);
        addRequirements(RobotContainer.targeting);

    }

    // This method is called once when command is started
    @Override
    public void initialize() {


    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // has no ball
        if(!RobotContainer.colour.isRightArtifactPresent()&& !RobotContainer.colour.isLeftArtifactPresent()&&
                !RobotContainer.colour.isRampArtifactPresent()){
            RobotContainer.shooter.flywheelSpeed(0);

        }

        x = RobotContainer.odometry.getCurrentPos().getX()* 3.281; // m to feet
        y = RobotContainer.odometry.getCurrentPos().getY()*3.281;

        // not in launch zones
        if (!(x <= -0.5 * Math.abs(y) || x >= (0.5 * Math.abs(y)) + 2)){
            RobotContainer.shooter.flywheelSpeed(0);
        }

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    // if has ball and in launch zone
    public boolean isFinished() {
        if ((RobotContainer.colour.isRampArtifactPresent()||
                RobotContainer.colour.isLeftArtifactPresent()||
                RobotContainer.colour.isRightArtifactPresent())&&
                (x <= -0.5 * Math.abs(y) || x >= (0.5 * Math.abs(y)) + 2)){
            return true;
        }else {
            return false;
        }
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}