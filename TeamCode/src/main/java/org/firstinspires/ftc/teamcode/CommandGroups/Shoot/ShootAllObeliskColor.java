package org.firstinspires.ftc.teamcode.CommandGroups.Shoot;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Intake.AdvanceIntake;
import org.firstinspires.ftc.teamcode.Commands.Shoot.AimToShoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.Shoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.WaitForSpinup;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;


// shoot all artifacts in order of obelisk pattern (if possible)
// typically used in auto
public class ShootAllObeliskColor extends SequentialCommandGroup {

    // constructor
    public ShootAllObeliskColor() {

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

                // line up ready to shoot obelisk colour #1
                new AimToShoot(RobotContainer.targeting.ShootObelisk1()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot obelisk colour #1
                new Shoot(RobotContainer.targeting.ShootObelisk1()),
                // block the shooter
                new InstantCommand(()-> RobotContainer.shotblock.Block()),
                // small pause for uptake to lower
                new Pause(0.25),

                // ARTIFACT #2

                // intake another artifact (advance the intake)
                new AdvanceIntake(),
                // unblock the shooter
                new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                // line up ready to shoot obelisk colour #2
                new AimToShoot(RobotContainer.targeting.ShootObelisk2()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot obelisk colour #2
                new Shoot(RobotContainer.targeting.ShootObelisk2()),

                // ARTIFACT #3

                // line up ready to shoot obelisk colour #3
                new AimToShoot(RobotContainer.targeting.ShootObelisk3()),
                // wait for shooter to spin up
                new WaitForSpinup(),
                // shoot obelisk colour #3
                new Shoot(RobotContainer.targeting.ShootObelisk3())



                //new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                // spin up
                //new InstantCommand(()-> RobotContainer.shooter.SetFlywheelSpeed(RobotContainer.targeting.CalculateSpeed())),
                // line up
                //new AimToShoot(),
                //pause for spin up
                //new Pause(1.0),
                // shoot artifact with color index=0
                //new ShootObeliskColor(0),
                // pause
                //new Pause(0.25),
                // intake another artifact
                //new IntakeCommand(),
                // shoot artifact with color index=1
                //new ShootObeliskColor(1),
                // pause
                //new Pause(0.25),
                // shoot artifact with color index=2
                //new ShootObeliskColor(2)
        );
    }

    // when command ends, or is interrupted, put block back on and ensure intake stopped
    @Override
    public void end(boolean interrupted){
        RobotContainer.shotblock.Block();
        RobotContainer.intake.intakeStop();
    }

}

