package org.firstinspires.ftc.teamcode.CommandGroups.Shoot;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoot.AimToShoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.ShootAnyColor;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

// shoot all artifacts regardless of colour
// typically used in teleop
public class ShootAllAnyColor extends SequentialCommandGroup {

    // constructor
    public ShootAllAnyColor() {

        addCommands (

                new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                // spin up
                new InstantCommand(()-> RobotContainer.shooter.flywheelSpeed(RobotContainer.targeting.CalculateSpeed())),
                // line up
                new AimToShoot(),
                // pause
                new Pause(1.0),
                // shoot any color
                new ShootAnyColor(),
                // pause
                new Pause(0.5),
                // intake another artifact
                new IntakeCommand(),

                new Pause(0.5),
                // shoot again
                new ShootAnyColor(),
                // pause
                new Pause(0.5),
                // shoot again
                new ShootAnyColor()
        );
    }
    @Override
    public void end(boolean interrupted){
        RobotContainer.shotblock.Block();
    }

}

