package org.firstinspires.ftc.teamcode.CommandGroups.Shoot;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Intake.AdvanceCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoot.AimToShootR1;
import org.firstinspires.ftc.teamcode.Commands.Shoot.Shoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.WaitForSpinup;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

// shoot all artifacts regardless of colour
// typically used in teleop
public class ShootAllAnyColor extends SequentialCommandGroup {

    // constructor
    public ShootAllAnyColor() {

        // this command directly commands the shooter speed
        // use to interrupt the default shooter command
        addRequirements(RobotContainer.shooter);

        addCommands (

                // ARTIFACT #1

                // spin up shooter to required speed
                new InstantCommand(()->RobotContainer.shooter.SetFlywheelSpeed(RobotContainer.targeting.CalculateSpeed())),
                // line up ready to shoot any colour
                new AimToShootR1(RobotContainer.targeting.ShootAny()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot any color
                new Shoot(RobotContainer.targeting.ShootAny()),
                // small pause for uptake to lower
                new Pause(0.25),

                // ARTIFACT #2

                // intake another artifact (advance the intake)
                new AdvanceCommand(),
                // line up ready to shoot any colour
                new AimToShootR1(RobotContainer.targeting.ShootAny()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot any color
                new Shoot(RobotContainer.targeting.ShootAny()),

                // ARTIFACT #3

                // line up ready to shoot any colour
                new AimToShootR1(RobotContainer.targeting.ShootAny()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot any color
                new Shoot(RobotContainer.targeting.ShootAny())

                // shoot any color
                //new ShootAnyColor(),
                // pause
                //new Pause(0.5),
                // intake another artifact
                //new IntakeCommand(),
                //new Pause(0.5),
                // shoot again
                //new ShootAnyColor(),
                // pause
                //new Pause(0.5),
                // shoot again
                //new ShootAnyColor()
        );
    }

}

