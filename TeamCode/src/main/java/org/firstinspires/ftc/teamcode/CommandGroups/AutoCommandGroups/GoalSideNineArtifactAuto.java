package org.firstinspires.ftc.teamcode.CommandGroups.AutoCommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.FastShootObeliskColor;
import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.ShootAllObeliskColor;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Drive.TurnTo;
import org.firstinspires.ftc.teamcode.Commands.Intake.HuntModeAuto;
import org.firstinspires.ftc.teamcode.Commands.Intake.HuntModeCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.JogBackIntake;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

import java.util.ArrayList;

public class GoalSideNineArtifactAuto extends SequentialCommandGroup {

    // Constructor
    public GoalSideNineArtifactAuto() {

        addCommands (

                new InstantCommand(()-> RobotContainer.odometry.setCurrentPos(AutoFunctions.redVsBlue(new Pose2d(-1.29, -1.29, new Rotation2d(Math.toRadians(-45.0)))))), // was x = -1.2, y = -1.35

                // Move to a shot #1
                new MoveToPose(
                        1.5,
                        0.8,
                        AutoFunctions.redVsBlue((new Pose2d(-0.45, -0.45, new Rotation2d(Math.toRadians(45.0)))))),// + or - 20 degrees // was -0.6, -0.6

//      -------------------------- Artifact Cycle #1 --------------------------
                new FastShootObeliskColor(),

                // Turn to Hunt
                new TurnTo(AutoFunctions.redVsBlue(-90.0),false,1.5),

                // Hunt
                new HuntModeAuto(3.0),

                // Clean-up Hunt
                new JogBackIntake(),

                // Move to shot #2
                new MoveToPose(
                        1.5,
                        0.8,
                        AutoFunctions.redVsBlue((new Pose2d(-0.6, -0.6, new Rotation2d(Math.toRadians(45.0)))))),

//      -------------------------- Artifact Cycle #2 --------------------------
                new FastShootObeliskColor(),

                // Move to pickup
                new MoveToPose(
                        1.5,
                        0.8,
                        AutoFunctions.redVsBlue((new Pose2d(0.3, -0.6, new Rotation2d(Math.toRadians(-90.0)))))),

                // Hunt
                new HuntModeAuto(3.0),

                // Clean-up Hunt
                new JogBackIntake(),

                // Follow a path around the gate to shot #3
//                new FollowPath(
//                        1.5,
//                        0.7,
//                        0.0,
//                        0.0,
//                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))),
//                        new ArrayList<Translation2d>() {{AutoFunctions.redVsBlue(new Translation2d(0.9, 0.0));}},
//                        AutoFunctions.redVsBlue(new Pose2d(-0.6, -0.6, new Rotation2d(Math.toRadians(150.0)))),
//                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(45)))),
                new MoveToPose(
                        1.5,
                        0.8,
                        AutoFunctions.redVsBlue(new Pose2d(-0.6, -0.6, new Rotation2d(Math.toRadians(45))))
                ),

//      -------------------------- Artifact Cycle #3 --------------------------
                new FastShootObeliskColor(),

                // Move off line
                new MoveToPose(
                        1.5,
                        0.8,
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