package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Intake.HuntMode.HuntModeAuto;
import org.firstinspires.ftc.teamcode.Commands.UptakeRampControle;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class AutoIntakeSequence extends SequentialCommandGroup {
    double default_timeout;
    // constructor
    public AutoIntakeSequence() {
        default_timeout = 1.75;
        addCommands (
                new ParallelCommandGroup(
                        new HuntModeAuto(default_timeout),
                        new UptakeRampControle(default_timeout)
                )
        );
    }

    public AutoIntakeSequence(double timeout) {

        addCommands (
             new ParallelCommandGroup(
                     new HuntModeAuto(timeout),
                     new UptakeRampControle(timeout)
             )
        );
    }
}
