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
public class ShootAllAnyColor extends SequentialCommandGroup {

    // constructor
    public ShootAllAnyColor() {

        // this command directly commands the shooter speed
        // use to interrupt the default shooter command
        addRequirements(RobotContainer.shooter);

        addCommands (

                // GET READY FOR SHOOTING

                // unblock the shooter
                new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                // spin up shooter to required speed
                new InstantCommand(()->RobotContainer.shooter.SetFlywheelSpeed(RobotContainer.targeting.CalculateSpeed())),

                // ARTIFACT #1

                // line up ready to shoot any colour
                new AimToShoot(RobotContainer.targeting.ShootAny()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot any color
                new Shoot(RobotContainer.targeting.ShootAny()),
                // block the shooter
                new InstantCommand(()-> RobotContainer.shotblock.Block()),
                // small pause for uptake to lower & blocker to engage
                new Pause(0.25),

                // ARTIFACT #2

                // intake another artifact (advance the intake)
                new AdvanceIntake(),
                // unblock the shooter
                new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                // line up ready to shoot any colour
                //new AimToShoot(RobotContainer.targeting.ShootAny()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot any color
                new Shoot(RobotContainer.targeting.ShootAny()),
                // small pause for uptake to lower
                new Pause(0.25),

                // ARTIFACT #3

                // line up ready to shoot any colour
                //new AimToShoot(RobotContainer.targeting.ShootAny()),
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

    // when command ends, or is interrupted, put block back on and ensure intake stopped
    @Override
    public void end(boolean interrupted){
        RobotContainer.shotblock.Block();
        RobotContainer.intake.intakeStop();
        RobotContainer.uptake.LowerLeftUptake();
        RobotContainer.uptake.LowerRightUptake();
    }

}

