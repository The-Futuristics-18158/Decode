package org.firstinspires.ftc.teamcode.CommandGroups.Shoot;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoot.AimToShoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.ShootObeliskColor;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

// shoot all artifacts in order of obelisk pattern (if possible)
// typically used in auto
public class ShootAllObeliskColor extends SequentialCommandGroup {

    // constructor
    public ShootAllObeliskColor() {

        addCommands (
                // spin up
                new InstantCommand(()-> RobotContainer.shooter.flywheelSpeed(RobotContainer.targeting.CalculateSpeed())),
                // line up
                new AimToShoot(),
                //pause for spin up
                new Pause(1.0),
                // shoot artifact with color index=0
                new ShootObeliskColor(0),
                // pause
                new Pause(0.25),
                // intake another artifact
                new IntakeCommand(),
                // shoot artifact with color index=1
                new ShootObeliskColor(1),
                // pause
                new Pause(0.25),
                // shoot artifact with color index=2
                new ShootObeliskColor(2)
        );
    }

}

