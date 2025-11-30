package org.firstinspires.ftc.teamcode.CommandGroups.Uptake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Utility.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class SaftyLowerUptake extends SequentialCommandGroup {

    // constructor
    public SaftyLowerUptake() {

        addCommands (
                new ParallelCommandGroup(new InstantCommand(()-> RobotContainer.uptake.LowerRightUptake()),new InstantCommand(()-> RobotContainer.uptake.LowerLeftUptake()))

        );
    }

}