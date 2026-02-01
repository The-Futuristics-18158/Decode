package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleRightUptake;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.GoalTargeting;


// This command shoots a SINGLE artifact of ANY COLOR
public class Shoot extends CommandBase {

    // command to shooter either left or right-side (to be determined in this module)
    Command cmd;

    // our shooting solution lambdA
    GoalTargeting.LeftVsRight solution;

    // constructor
    public Shoot(GoalTargeting.LeftVsRight solution) {
        // by default, do nothing unless chosen otherwise
        cmd=null;
        this.solution = solution;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // get the side from our target solution
        GoalTargeting.ShootSide side = solution.getSide();

        // Logic to shoot ball
        if (side== GoalTargeting.ShootSide.LEFT)
            cmd = new CycleLeftUptake();

        else if (side== GoalTargeting.ShootSide.RIGHT)
            cmd = new CycleRightUptake();

        else if (side== GoalTargeting.ShootSide.BOTH)
            cmd = new ParallelCommandGroup(new CycleLeftUptake(), new CycleRightUptake());

        else
            cmd = null;

        // initialize shoot command
        if (cmd!=null)
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