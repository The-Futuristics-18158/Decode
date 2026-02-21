package org.firstinspires.ftc.teamcode.CommandGroups.Shoot.Fast;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleRightUptake;
import org.firstinspires.ftc.teamcode.Commands.Drive.ManualDriveAutoTurnToTarget;
import org.firstinspires.ftc.teamcode.Commands.Drive.WaitForTarget;
import org.firstinspires.ftc.teamcode.Commands.Shoot.WaitForSpinup;
import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Obelisk;


// command template
public class FastShootObeliskColorConstantAim extends CommandBase {

    // the sequential command to shoot
    CommandGroupBase cmd;

    // constructor
    public FastShootObeliskColorConstantAim() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.hoodtilt);
        addRequirements(RobotContainer.intake);
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

        // wait for robot to finish turning to target and spinning up flywheel, whichever lasts longer
        cmd.addCommands(new ParallelCommandGroup(
                    //new TurnToTarget(4.0),
                    new WaitForTarget(3.0),
                    new WaitForSpinup()
        ));


        // ---------- Artifact #1 ----------

        // we are shooting on left unless set otherwise
        boolean shotleft = true;

        // get first desired color
        Obelisk.ArtifactColor color1 = RobotContainer.obelisk.GetColorAtIndex(0);

        // if color is on left
        if (RobotContainer.artifactCamera.getRightColour().name().equals(color1.name()))
            cmd.addCommands(new CycleLeftUptake());

        // if color is on right
        else if (RobotContainer.artifactCamera.getLeftColour().name().equals(color1.name()))
            { cmd.addCommands(new CycleRightUptake()); shotleft = false; }

        // no matches - or we don't know - shoot left
        else
            cmd.addCommands(new CycleLeftUptake());

        // advance next ball
        AddAdvance();

        // allow extra time for spinup if we are shooting from far away
        if (RobotContainer.targeting.GetDistanceToGoal() >= 2.75)
            cmd.addCommands(new Pause(0.15));


        // ---------- Artifact #2 ----------

        // get second desired color
        Obelisk.ArtifactColor color2 = RobotContainer.obelisk.GetColorAtIndex(1);

        // if we previously shot left, and right matches 2nd desired color, shoot right
        if (shotleft && RobotContainer.artifactCamera.getLeftColour().name().equals(color2.name()))
            cmd.addCommands(new CycleRightUptake());

        // previously shot right and if left matches 2nd desired color, shoot left
        else if (!shotleft && RobotContainer.artifactCamera.getRightColour().name().equals(color2.name()))
            cmd.addCommands(new CycleLeftUptake());

        // the 'alternate side does not match next colour, so we need to shoot the ball we advanced
        else
        {
            // did we shoot the left first, if so shoot left again, otherwise shoot right
            if (shotleft)
                cmd.addCommands(new CycleLeftUptake());
            else
                cmd.addCommands(new CycleRightUptake());
        }

        // delay to allow flywheel spinup
        if (RobotContainer.targeting.GetDistanceToGoal() <= 2.75){
            cmd.addCommands(new Pause(0.2));
        }else {
            cmd.addCommands(new Pause(0.4));
        }


        // ---------- Artifact #3 ----------

        // shoot both sides - don't care which side ball is in

        // cycle both sides
        cmd.addCommands(new ParallelCommandGroup(
                new CycleRightUptake(),
                new CycleLeftUptake() ));


        // ---------- Overall parallel race group command ----------

        // put command into a parallel race group with manual driving with auto rotate to target
        cmd = new ParallelRaceGroup(cmd, new ManualDriveAutoTurnToTarget());

        // initialize the command
        cmd.initialize();
    }

    // helper function - adds advance to the provided command sequence
    private void AddAdvance()
    {
        cmd.addCommands(new InstantCommand(()-> RobotContainer.shotblock.Block()));
        cmd.addCommands(new InstantCommand(()->RobotContainer.intake.intakeRun()));
        cmd.addCommands(new Pause(0.35));
        cmd.addCommands(new ParallelCommandGroup(
                new InstantCommand(()->RobotContainer.intake.intakeStop()),
                new InstantCommand(()-> RobotContainer.shotblock.Unblock())  ));
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
        // chosen not to call end. FTC library has bug that may cause crash.
        // instead, we simply turn off all affected subsystems
        //cmd.end(interrupted);

        RobotContainer.shotblock.Block();
        RobotContainer.intake.intakeStop();
        RobotContainer.uptake.LowerLeftUptake();
        RobotContainer.uptake.LowerRightUptake();
        RobotContainer.shooter.SetFlywheelSpeed(0);
        RobotContainer.hoodtilt.SetHoodPosition(0.0);
    }

}