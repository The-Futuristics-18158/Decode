package org.firstinspires.ftc.teamcode.CommandGroups.Shoot.Fast;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleRightUptake;
import org.firstinspires.ftc.teamcode.Commands.Drive.TurnToTarget;
import org.firstinspires.ftc.teamcode.Commands.Shoot.WaitForSpinup;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Cameras.ArtifactCamera;


// Command
public class FastShootPurple extends CommandBase {

    // The sequential command that we are creating and running
    SequentialCommandGroup cmd;


    // constructor
    public FastShootPurple() {

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
        if(RobotContainer.artifactCamera.getRightColour().name().equals(ArtifactCamera.ArtifactColours.Purple.name()))
            cmd.addCommands(new CycleLeftUptake());
        else if (RobotContainer.artifactCamera.getLeftColour().name().equals(ArtifactCamera.ArtifactColours.Purple.name()))
            cmd.addCommands(new CycleRightUptake());
        else if(RobotContainer.artifactCamera.IsRightPresent())
            cmd.addCommands(new CycleLeftUptake());
        else if (RobotContainer.artifactCamera.IsLeftPresent())
            cmd.addCommands(new CycleRightUptake());
        else
            cmd.addCommands(new ParallelCommandGroup(
                    new CycleRightUptake(),
                    new CycleLeftUptake()
            ));

        cmd.addCommands(new Pause(0.05));

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