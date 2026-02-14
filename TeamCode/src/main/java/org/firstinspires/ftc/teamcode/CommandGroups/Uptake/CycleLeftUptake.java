package org.firstinspires.ftc.teamcode.CommandGroups.Uptake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class CycleLeftUptake extends SequentialCommandGroup {

    // constructor
    public CycleLeftUptake() {

        addCommands (
                // Only use commented out lines when doing shot charachterization
                //new InstantCommand(()-> RobotContainer.shotblock.Unblock()),
                //new Pause(0.1),

                new InstantCommand(()-> RobotContainer.uptake.RaiseLeftUptake()),

                new Pause(0.2),

                new InstantCommand(()-> RobotContainer.uptake.LowerLeftUptake())

                //new InstantCommand(()-> RobotContainer.shotblock.Block())

        );
    }

    @Override
    public void end(boolean interrupted){
        RobotContainer.uptake.LowerLeftUptake();
        //RobotContainer.shotblock.Block();
    }

}