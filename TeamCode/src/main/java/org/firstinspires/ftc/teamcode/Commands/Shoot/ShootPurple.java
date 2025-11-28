package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleRightUptake;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.ColourSensor;


// This command shoots a SINGLE artifact of ANY COLOR
public class ShootPurple extends CommandBase {

    // command to shooter either left or right-side (to be determined in this module)
    Command cmd;

    // constructor
    public ShootPurple() {
        // by default, do nothing unless chosen otherwise
        cmd=null;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // Logic to shoot ball
        if(RobotContainer.colour.GetLeftColour().name().equals(ColourSensor.ArtifactColours.Purple.name())){
            cmd = new CycleLeftUptake();

        } else if (RobotContainer.colour.GetRightColour().name().equals(ColourSensor.ArtifactColours.Purple.name())) {
            cmd = new CycleRightUptake();

        }else if(RobotContainer.colour.isLeftArtifactPresent()){
            cmd = new CycleLeftUptake();

        }else if (RobotContainer.colour.isRightArtifactPresent()) {
            cmd = new CycleRightUptake();

        }else{
            cmd = new ParallelCommandGroup(new CycleLeftUptake(), new CycleRightUptake());
        }

        // initialize shoot command
        cmd.initialize();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        if (cmd!=null)
            cmd.execute();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        if (cmd!=null)
            return cmd.isFinished();
        else
            return true;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        if (cmd!=null)
            cmd.end(interrupted);
    }

}