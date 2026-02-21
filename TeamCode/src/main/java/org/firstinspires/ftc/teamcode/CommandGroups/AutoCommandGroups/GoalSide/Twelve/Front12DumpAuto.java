package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups.GoalSide.Twelve;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.Fast.FastShootObeliskColor;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Drive.TurnTo;
import org.firstinspires.ftc.teamcode.Commands.Intake.HuntMode.HuntModeAuto;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

public class Front12DumpAuto extends SequentialCommandGroup {

    // Constructor
    public Front12DumpAuto() {

        addCommands (
                new InstantCommand(()-> RobotContainer.odometry.setCurrentPos(AutoFunctions.redVsBlue(new Pose2d(-1.29, -1.29, new Rotation2d(Math.toRadians(-45.0)))))), // was x = -1.2, y = -1.35
//      -------------------------- Artifact Cycle #1  --------------------------
                // Move to a shot #1
                new MoveToPose(
                        1.5,
                        4.0,
                        AutoFunctions.redVsBlue((new Pose2d(-0.3, -0.3, new Rotation2d(Math.toRadians(45.0)))))),// + or - 20 degrees // was -0.6, -0.6

                // Shot #1
                new FastShootObeliskColor(),

//      -------------------------- Start of Artifact Cycle #2 --------------------------
                // Turn to Hunt
                new TurnTo(AutoFunctions.redVsBlue(-90.0),false,1.5),

                // Hunt
                new HuntModeAuto(3.0),

//    -------------------------- Start Of Dump --------------------------

                // Line up to dump gate
                new MoveToPose(
                        1.2,
                        4.0,
                        AutoFunctions.redVsBlue((new Pose2d(-0.3, -1.2, new Rotation2d(Math.toRadians(0.0)))))),

                // Dump Gate
                new MoveToPose(
                        1.0,
                        1.2,
                        AutoFunctions.redVsBlue((new Pose2d(-0.3, -1.4, new Rotation2d(Math.toRadians(0.0)))))),

//    -------------------------- End Of Dump --------------------------

                // Move to shot #2
                new MoveToPose(
                        1.5,
                        4.0,
                        AutoFunctions.redVsBlue((new Pose2d(-0.6, -0.6, new Rotation2d(Math.toRadians(45.0)))))),

                // Shot #2
                new FastShootObeliskColor(),

//    -------------------------- Start of Artifact Cycle #3 --------------------------
                // Move to pickup
                new MoveToPose(
                        1.5,
                        3.5,
                        AutoFunctions.redVsBlue((new Pose2d(0.3, -0.6, new Rotation2d(Math.toRadians(-90.0)))))),

                // Hunt
                new HuntModeAuto(3.0),

                // Move to Shot #3
                new MoveToPose(
                        1.5,
                        4.0,
                        AutoFunctions.redVsBlue(new Pose2d(-0.6, -0.6, new Rotation2d(Math.toRadians(45))))
                ),

                // Shot #3
                new FastShootObeliskColor(),

//              // Move to intake point
                new MoveToPose(
                        1.5,
                        4.0,
                        AutoFunctions.redVsBlue((new Pose2d(0.98, -0.6, new Rotation2d(Math.toRadians(-90.0)))))),

                new HuntModeAuto(3.0),

                // Move to shot #4
                new MoveToPose(
                        1.5,
                        4.0,
                        AutoFunctions.redVsBlue(new Pose2d(-0.6, -0.6, new Rotation2d(Math.toRadians(45))))),

//      -------------------------- Artifact Cycle #4 --------------------------
                new FastShootObeliskColor(),

                // Move off the line
                new MoveToPose(
                        1.5,
                        4.0,
                        AutoFunctions.redVsBlue((new Pose2d(0.0, -0.9, new Rotation2d(Math.toRadians(180.0))))))
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