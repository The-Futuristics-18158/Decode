package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.CommandGroups.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.CycleRightUptake;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.ColourSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Obelisk;


// This command shoots a SINGLE artifact according to INDEX of OBELISK pattern
public class ShootObeliskColor extends CommandBase {

    // command to shooter either left or right-side (to be determined in this module)
    Command cmd;

    // obelisk color index to shoot (=0,1 or 2)
    int colorIndex;

    // constructor
    // input: index of color (0,1,2)
    public ShootObeliskColor(int index) {
        // by default, do nothing unless chosen otherwise
        cmd=null;

        // make sure index is valid, or else assume index of 0 as default
        if (index>=0 && index<=2)
            colorIndex=index;
        else
            index=0;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // get desired obelisk color
        Obelisk.ArtifactColor color = RobotContainer.obelisk.GetColorAtIndex(colorIndex);

        // insert logic here to shoot ball

        if (color.name().equals(RobotContainer.colour.GetLeftColour().name()))
        // if desired obelisk colour matches colour reported by left colour sensor then
            // cycle left side
            cmd = new CycleLeftUptake();

        // else desired obelisk colour matches colour reported by right colour sensor then
        //  cycle right side

        // we don't have desired color
        // else if left has artifact of any color
        // cycle left side


        // else if right has artifact of any color
        // cycle right side

        // maybe we have artifact but sensor not working
        // else
        // cycle both sides just in case
        cmd = new ParallelCommandGroup(new CycleLeftUptake(), new CycleRightUptake());


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