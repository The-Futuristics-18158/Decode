package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups.LoadingSide.Nine.FarShot;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.BackNineManualHunt;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.Fast.FastShootObeliskColor;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Intake.HuntModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeRunEndless;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

public class Back9LongShotsAuto extends SequentialCommandGroup {

    // Constructor
    public Back9LongShotsAuto() {

        addCommands (
                // Was X = 1.59 Y Was -0.39
                new InstantCommand(()-> RobotContainer.odometry.setCurrentPos(AutoFunctions.redVsBlue(new Pose2d(1.60, -0.37, new Rotation2d(Math.toRadians(0.0)))))),

                // Move to shoot
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.34, -0.38, new Rotation2d(Math.toRadians(23.0)))))), // + or - 20 degrees

//      -------------------------- Artifact Cycle #1 --------------------------
                new FastShootObeliskColor(),

//              // Move to intake point
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(0.98, -0.6, new Rotation2d(Math.toRadians(-90.0)))))),

//                // Hunt
//                new HuntModeAuto(4.0),
                new ParallelRaceGroup(

                        // Move with pause
                        new BackNineManualHunt(),

                        // intake untile has
                        new IntakeRunEndless()
               ),

                // Move to shoot
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.34, -0.38, new Rotation2d(Math.toRadians(23.0)))))), // + or - 20 degrees

//      -------------------------- Artifact Cycle #2 --------------------------
                new FastShootObeliskColor(),

                // Move to pickup
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.05, -1.59, new Rotation2d(Math.toRadians(0.0)))))),

                //
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.05, -1.62, new Rotation2d(Math.toRadians(-15.0)))))),

                new HuntModeCommand(3.0),

                // Move to shoot
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.34, -0.38, new Rotation2d(Math.toRadians(23.0)))))),

//      -------------------------- Artifact Cycle #3 --------------------------
                new FastShootObeliskColor(),

                // Leave
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.5, -0.9, new Rotation2d(Math.toRadians(0.0))))))
        );
    }
}

//Example MoveToPose
// new MoveToPose(
//           1.5,
//           1.0,
// AutoFunctions.redVsBlue(new Pose2d(-0.22, 1.2, new Rotation2d(Math.toRadians(-90))))
//        ),

//Example FollowPath
// new FollowPath(
//             2.0,
//             1.0,
//             0.0,
//             0.0,
// AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))),
// new ArrayList<Translation2d>() {{ }},
// AutoFunctions.redVsBlue(new Pose2d(0.55, 0.25, new Rotation2d(Math.toRadians(-180)))),
// AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-180)))),


// Example #2: Using conditional command
// conditionally run commands depending on condition
/*      new FourWayConditionalCommand(
            ()-> { return true; },
            new SequentialCommandGroup(
                commands to do this
            ),
            ()-> { return false; },
            new SequentialCommandGroup(
                commands to do that
            ),
            ()-> { return false; },
            new SequentialCommandGroup(
                commands to do those
            ),
            new SequentialCommandGroup(
                commands to do something else
            )

        ) // end FourWayCondition
*/