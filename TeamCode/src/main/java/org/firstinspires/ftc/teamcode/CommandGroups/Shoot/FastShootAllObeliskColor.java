package org.firstinspires.ftc.teamcode.CommandGroups.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleRightUptake;
import org.firstinspires.ftc.teamcode.Commands.Drive.TurnToTarget;
import org.firstinspires.ftc.teamcode.Commands.Shoot.AimToShoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.Shoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot.WaitForSpinup;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class FastShootAllObeliskColor extends CommandBase {

    // the sequential command that we are creating and running
    SequentialCommandGroup cmd;


    // constructor
    public FastShootAllObeliskColor() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.hoodtilt);
        addRequirements(RobotContainer.targeting);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // Create command sequence to be run
        cmd = new SequentialCommandGroup();

        // initial commands - unblock and set hood angle/speed
        // note: for these, we do not need to add into sequence since they are instant anyway
        // it is faster to simply execute now
        RobotContainer.shotblock.Unblock();
        RobotContainer.targeting.SetHoodAngleAndSpeed();

        // turn to target, wait for shooter spin up, whichever lasts longer
        cmd.addCommands(new ParallelCommandGroup(
                new TurnToTarget(10.0),
                new WaitForSpinup()
        ));

        // Artifact #1
        RobotContainer.targeting.ShootObelisk1();
        cmd.addCommands(new Pause(0.05));

        // Artifact #2
        RobotContainer.targeting.ShootObelisk2();

        // unblock
        cmd.addCommands(new InstantCommand(()-> RobotContainer.shotblock.Block()));

        // start intake
        cmd.addCommands(new InstantCommand(()->RobotContainer.intake.intakeRun()));
        cmd.addCommands(new Pause(0.15));

        // stop intake / unblock
        cmd.addCommands(new ParallelCommandGroup(
                new InstantCommand(()->RobotContainer.intake.intakeStop()),
                new InstantCommand(()-> RobotContainer.shotblock.Unblock())
        ));
        cmd.addCommands(new Pause(0.1));

        // ARTIFACT #3
        RobotContainer.targeting.ShootObelisk3();

        // initialize the sequence command
        cmd.initialize();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        cmd.execute();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        // we are finished only when the sequence is finished
        return cmd.isFinished();
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        // we are ending, end the sequence
        cmd.end(interrupted);

        RobotContainer.shotblock.Block();
        RobotContainer.intake.intakeStop();
        RobotContainer.uptake.LowerLeftUptake();
        RobotContainer.uptake.LowerRightUptake();
        RobotContainer.shooter.SetFlywheelSpeed(0);
        RobotContainer.hoodtilt.SetHoodPosition(0.0);
    }

}