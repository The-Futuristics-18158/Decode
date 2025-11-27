package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotContainer;

/*
 * This file contains an example of an "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 */
@TeleOp(name="Red TeleOp", group="OpMode")
//@Disabled
public class RedTeleop extends CommandOpMode {

    // Initialize all objects, set up subsystems, etc...
    @Override
    public void initialize() {

        // initialize robot
        // set team alliance color to red (isRedAlliance=true)
        RobotContainer.Init(this, true);

        // perform any teleop initialization
        RobotContainer.Init_TeleOp();

        // wait for start button
        waitForStart();

        // if start button has been pressed
        if (opModeIsActive()) {

            // ---------- teleop command ----------

            // add any command to run automatically at start of teleop
        }

    }

    // Run Op Mode. Is called after user presses play button
    // called continuously
    @Override
    public void run() {
        // execute robot periodic function
        RobotContainer.Periodic();
    }
}