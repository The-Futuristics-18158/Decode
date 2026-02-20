package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.Shoot.Fast.FastShootObeliskColor;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;

public class GhostMoves extends SequentialCommandGroup {

    // Constructor
    public GhostMoves() {

        addCommands (
//      -------------------------- Movement Cycle One  --------------------------

                // Move closer to intake box
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.52, -1.0, new Rotation2d(Math.toRadians(-90.0)))))),

                // Move straight into pile
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.52, -1.5, new Rotation2d(Math.toRadians(-90.0)))))),

                // Move straight back

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.52, -1.0, new Rotation2d(Math.toRadians(-90.0)))))),

//      -------------------------- End of Movement Cycle One --------------------------

                // Shift
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.4, -1.0, new Rotation2d(Math.toRadians(-90.0)))))),

//      -------------------------- Start of Movement Cycle Two --------------------------
                // Move straight into pile
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.4, -1.5, new Rotation2d(Math.toRadians(-90.0)))))),

                // Move straight back
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.4, -1.0, new Rotation2d(Math.toRadians(-90.0)))))),

//      -------------------------- End of Movement Cycle Two --------------------------

//      -------------------------- Movement Cycle Three  --------------------------

                // Move closer to intake box
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.52, -1.0, new Rotation2d(Math.toRadians(-90.0)))))),

                // Move straight into pile
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.52, -1.5, new Rotation2d(Math.toRadians(-90.0)))))),

                // Move straight back

                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.52, -1.0, new Rotation2d(Math.toRadians(-90.0)))))),

//      -------------------------- End of Movement Cycle Three --------------------------

                // Shift
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.4, -1.0, new Rotation2d(Math.toRadians(-90.0)))))),

//      -------------------------- Start of Movement Cycle Four --------------------------
                // Move straight into pile
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.4, -1.5, new Rotation2d(Math.toRadians(-90.0)))))),

                // Move straight back
                new MoveToPose(
                        1.5,
                        1.0,
                        AutoFunctions.redVsBlue((new Pose2d(1.4, -1.0, new Rotation2d(Math.toRadians(-90.0))))))

//      -------------------------- End of Movement Cycle Four --------------------------



        );
    }
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.FieldDrive(0.0, 0.0, 0.0);
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