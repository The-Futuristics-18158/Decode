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
public class ShootSingleGreen extends SequentialCommandGroup {

    // constructor
    public ShootSingleGreen() {

        // this command directly commands the shooter speed
        // use to interrupt the default shooter command
        addRequirements(RobotContainer.shooter);

        addCommands (

                // GET READY FOR SHOOTING

                // unblock the shooter
                new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                // spin up shooter to required speed
                new InstantCommand(()->RobotContainer.targeting.SetHoodAngleAndSpeed()),
                // ARTIFACT #1 (GREEN)

                // line up ready to shoot green
                new AimToShoot(RobotContainer.targeting.ShootGreen()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot green
                new Shoot(RobotContainer.targeting.ShootGreen()),
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
                //new Pause(1.0),
                // shoot any color
                //new ShootSingleGreen(),
                // pause
                //new Pause(0.5),
                // intake another artifact
                //new IntakeCommand(),
                //new Pause(0.5)

        );
    }

    // when command ends, or is interrupted, put block back on and ensure intake stopped
    @Override
    public void end(boolean interrupted){
        RobotContainer.shotblock.Block();
        RobotContainer.intake.intakeStop();
        RobotContainer.uptake.LowerLeftUptake();
        RobotContainer.uptake.LowerRightUptake();
        RobotContainer.hoodtilt.SetHoodPosition(0.0);
        RobotContainer.shooter.SetFlywheelSpeed(0.0);
    }

}

