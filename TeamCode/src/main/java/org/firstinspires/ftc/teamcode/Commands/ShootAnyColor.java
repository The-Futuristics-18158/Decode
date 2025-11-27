package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandGroups.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.CycleRightUptake;


// This command shoots a SINGLE artifact of ANY COLOR
public class ShootAnyColor extends CommandBase {

    // command to shooter either left or right-side (to be determined in this module)
    Command cmd;

    // constructor
    public ShootAnyColor() {
        // by default, do nothing unless chosen otherwise
        cmd=null;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // insert logic here to shoot any ball, regardless of colour
        // if left side has a ball then:
        //      cycle left side  (use:  cmd = new CycleLeftUpdate();  )

        // else if right side as a ball then:
        //      cycle right side

        // else
        // we don't have a ball, or else sensor(s) not working
        // shoot both sides simultaneously in case one side has a ball but bad sensor
        //cmd = new ParallelCommandGroup(new CycleLeftUptake(), new CycleRightUptake());


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