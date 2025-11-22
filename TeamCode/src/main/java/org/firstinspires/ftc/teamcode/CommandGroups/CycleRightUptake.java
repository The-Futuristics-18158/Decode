package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class CycleRightUptake extends SequentialCommandGroup {

    // constructor
    public CycleRightUptake() {

        addCommands (
                new InstantCommand(()-> RobotContainer.uptake.RaiseRightUptake()),

                new Pause(0.2),

                new InstantCommand(()-> RobotContainer.uptake.LowerRightUptake())

        );
    }

}