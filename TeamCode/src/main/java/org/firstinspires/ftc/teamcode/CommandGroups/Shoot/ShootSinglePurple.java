package org.firstinspires.ftc.teamcode.CommandGroups.Shoot;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Intake.AdvanceIntake;
import org.firstinspires.ftc.teamcode.Commands.Shoot.AimToShoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.Shoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.WaitForSpinup;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;


// shoot all artifacts regardless of colour
// typically used in teleop
public class ShootSinglePurple extends SequentialCommandGroup {

    // constructor
    public ShootSinglePurple() {

        // this command directly commands the shooter speed
        // use to interrupt the default shooter command
        addRequirements(RobotContainer.shooter);

        addCommands (

                // GET READY FOR SHOOTING

                // unblock the shooter
                new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                // spin up shooter to required speed
                new InstantCommand(()->RobotContainer.shooter.SetFlywheelSpeed(RobotContainer.targeting.CalculateSpeed())),

                // ARTIFACT #1 (PURPLE)

                // line up ready to shoot purple
                new AimToShoot(RobotContainer.targeting.ShootPurple()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot purple
                new Shoot(RobotContainer.targeting.ShootPurple()),
                // block the shooter
                new InstantCommand(()-> RobotContainer.shotblock.Block()),
                // small pause for uptake to lower
                new Pause(0.25),
                // intake another artifact (advance the intake)
                new AdvanceIntake(),
                // small pause
                new Pause(0.25)


                //new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                // spin up
                //new InstantCommand(()-> RobotContainer.shooter.SetFlywheelSpeed(RobotContainer.targeting.CalculateSpeed())),
                // line up
                //new AimToShoot(),
                // wait for spin up
                //new Pause(0.8),
                // shoot any color
                //new ShootPurple(),
                // pause
                //new Pause(0.5),
                // intake another artifact
                //new IntakeCommand(),
                //new Pause(0.5)

        );
    }

    // when command ends, or is interrupted, put block back on
    @Override
    public void end(boolean interrupted){
        RobotContainer.shotblock.Block();
    }

}

