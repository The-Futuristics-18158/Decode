package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups.LoadingSide.Twelve;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class LoadingSideEmptyTwelveArtifactAuto extends SequentialCommandGroup {

    // Constructor
    //public LoadingSideEmptyTwelveArtifactAuto() {
//
//        addCommands (
//                // Was X = 1.59 Y was -0.39
//                new InstantCommand(()-> RobotContainer.odometry.setCurrentPos(AutoFunctions.redVsBlue(new Pose2d(1.64, -0.35, new Rotation2d(Math.toRadians(0.0)))))),
//
//                // Move to shot #1
//                new MoveToPose(
//                        1.75,
//                       1.0,
//                        AutoFunctions.redVsBlue((new Pose2d(-0.3, -0.3, new Rotation2d(Math.toRadians(45.0)))))), // + or - 20 degrees
//
//      -------------------------- Artifact Cycle #1 --------------------------
//                new FastShootObeliskColor(),
//
//                // Move to pickup
//                new MoveToPose(
//                        1.5,
//                        0.7,
//                        AutoFunctions.redVsBlue((new Pose2d(0.3, -0.6, new Rotation2d(Math.toRadians(-90.0)))))),
//
//                // Hunt
//                new HuntModeCommand(2.5),
//
//                // Clean-up Hunt
//                new JogBackIntake(),
//
//                // Move to dump
//                // Move to shoot
//                // Move to pickup
//
//
//                // Hunt
//                new HuntModeCommand(2.5),
//                // Clean-up Hunt
//                new JogBackIntake(),
//
//                // Follow a path around the gate to shot #2
//                new FollowPath(
//                        1.5,
//                        1.0,
//                        0.0,
//                        0.0,
//                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(90.0))),
//                        new ArrayList<Translation2d>(){{AutoFunctions.redVsBlue(new Translation2d(0.5,-0.95));}},
//                        AutoFunctions.redVsBlue(new Pose2d(-0.3, -0.3, new Rotation2d(Math.toRadians(180)))),
//                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(45.0)))),
//
////      -------------------------- Artifact Cycle #2 --------------------------
//                new FastShootObeliskColor(),
//
//                // Move to pickup
//                new MoveToPose(
//                        1.5,
//                        1.0,
//                        AutoFunctions.redVsBlue((new Pose2d(0.3, -0.6, new Rotation2d(Math.toRadians(-90.0)))))),
//
//                // Hunt
//                new HuntModeCommand(2.5),
//                // Clean-up Hunt
//                new JogBackIntake(),
//                // Move to shot #3
//                new MoveToPose(
//                        1.5,
//                        1.0,
//                        AutoFunctions.redVsBlue((new Pose2d(-0.3, -0.3, new Rotation2d(Math.toRadians(45.0)))))),
//
////      -------------------------- Artifact Cycle #3 --------------------------
//                new FastShootObeliskColor(),
//                // Move off line
//                new MoveToPose(
//                        1.5,
//                        1.5,
//                        AutoFunctions.redVsBlue((new Pose2d(-0.3, -0.67, new Rotation2d(Math.toRadians(-90.0))))))
//
//        );
//    }
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