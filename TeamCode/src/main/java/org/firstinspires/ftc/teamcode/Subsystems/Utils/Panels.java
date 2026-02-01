package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.lights.PanelsLights;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.RobotContainer;


public class Panels extends SubsystemBase {

    // Local objects and variables here
    public TelemetryManager Telemetry;
    public TelemetryManager.TelemetryWrapper FTCTelemetry;
    public FieldManager Field;
    public PanelsGamepad Gamepad;
    public PanelsLights Lights;
    //public PanelsCameraStream CameraStream;

    // field update counter
    private int FieldUpdateCounter;

    public Panels() {

        // get pointers to various parts of Panels
        Telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        FTCTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        Field = PanelsField.INSTANCE.getField();
        Gamepad = PanelsGamepad.INSTANCE;
        Lights = PanelsLights.INSTANCE;
        //CameraStream = PanelsCameraStream.INSTANCE;

        FieldUpdateCounter=0;
    }

    @Override
    public void periodic() {

        // update field for robot position - only update once every 10 iterations of periodic
        FieldUpdateCounter++;
        if (FieldUpdateCounter>=10)
        {
            FieldUpdateCounter=0;
            UpdateField();
        }
        // code for later
        // only update dashboard and controller telemetry if opmode not about to be shut down
        //            if (!RobotContainer.ActiveOpMode.isStopRequested() && updateDashboardCounter >= 15)
    }

    private void UpdateField() {

        // set field offset to FTC default
        Field.setOffsets(FieldPresets.INSTANCE.getDEFAULT_FTC());

        // robot outline (note: values intentionally left in inches)
        // 0,0 is center of robot
        Translation2d p1 = new Translation2d(9, 0);
        Translation2d p2 = new Translation2d(-9, 9);
        Translation2d p3 = new Translation2d(-9, -9);
        Translation2d p4 = new Translation2d(0,0);

        // set line style depending on red or blue team
        if (RobotContainer.isRedAlliance())
            Field.setStyle("none", "red", 1.0);
        else
            Field.setStyle("none", "blue", 1.0);

        // get current odometry
        Pose2d pose = RobotContainer.odometry.getCurrentPos();

        // Robot position in inches - get translation and then scale from m to inches
        Translation2d xy = (pose.getTranslation()).times(39.3701);
        Rotation2d angle = pose.getRotation();

        // rotate outline by angle of odometry and then add x,y position offset
        Translation2d p1rotated=p1.rotateBy(angle).plus(xy);
        Translation2d p2rotated=p2.rotateBy(angle).plus(xy);
        Translation2d p3rotated=p3.rotateBy(angle).plus(xy);
        Translation2d p4rotated=p4.rotateBy(angle).plus(xy);

        // draw robot outline
        Field.moveCursor(p1rotated.getX(), p1rotated.getY());
        Field.line(p2rotated.getX(), p2rotated.getY());
        Field.moveCursor(p2rotated.getX(), p2rotated.getY());
        Field.line(p3rotated.getX(), p3rotated.getY());
        Field.moveCursor(p3rotated.getX(), p3rotated.getY());
        Field.line(p1rotated.getX(), p1rotated.getY());
        // draw circle at front of robot
        Field.moveCursor(p1rotated.getX(), p1rotated.getY());
        Field.circle(1.5);
        // draw dot in middle of robot
        Field.moveCursor(p4rotated.getX(), p4rotated.getY());
        Field.circle(0.40);

        // do we have a trajectory to plot?
        //if (currentTrajectoryXpoints!=null && currentTrajectoryYpoints!=null)
        //    field.fieldOverlay().strokePolyline(currentTrajectoryXpoints, currentTrajectoryYpoints);

        // update field
        Field.update();
    }


    // variables used for displaying paths on dashboard field widget
    // arrays hold x and y points of currently shown path(s)
    // arrays are set to null if no path to be shown
    private double[] currentTrajectoryXpoints;
    private double[] currentTrajectoryYpoints;

    public void DisplayTrajectory (Trajectory trajectory) {

        if (trajectory!=null)
        {
            // translate provided trajectory into separate x,y points array for use by dashboard field widget
            int length = trajectory.getStates().size();
            currentTrajectoryXpoints = new double[length];
            currentTrajectoryYpoints = new double[length];
            for (int index = 0; index < length; ++index) {
                currentTrajectoryXpoints[index] = 39.3701*trajectory.getStates().get(index).poseMeters.getX();
                currentTrajectoryYpoints[index] = 39.3701*trajectory.getStates().get(index).poseMeters.getY();
            }
        }
        else
        {
            // no trajectory to display - set arrays to null
            currentTrajectoryXpoints = null;
            currentTrajectoryYpoints = null;
        }
    }


    private void gamepadTelemetry(){

        // Get current gamepads from the active FTC hardware instance
        GamepadManager g1 = Gamepad.getFirstManager();
        //com.qualcomm.robotcore.hardware.Gamepad g2 = RobotContainer.toolOp.gamepad;

        //if (g1 == null || g2 == null) {
        //    panelsTelemetry.getTelemetry().debug("Gamepads not initialized yet!");
        //    panelsTelemetry.getTelemetry().update(telemetry);
        //    return;
        //}

        Telemetry.debug("==== Buttons ====");
        Telemetry.debug("A: " + g1.getCircle());
        //Telemetry.debug("B: " + g1.b);
        //Telemetry.debug("X: " + g1.x);
        //Telemetry.debug("Y: " + g1.y);
        //Telemetry.debug("DPad Up: " + g1.dpad_up);
        //Telemetry.debug("DPad Down: " + g1.dpad_down);
        //Telemetry.debug("DPad Left: " + g1.dpad_left);
        //Telemetry.debug("DPad Right: " + g1.dpad_right);
        //Telemetry.debug("Left Bumper: " + g1.left_bumper);
        //Telemetry.debug("Right Bumper: " + g1.right_bumper);
        //Telemetry.debug("Left Trigger: " + g1.left_trigger);
        //Telemetry.debug("Right Trigger: " + g1.right_trigger);
        //Telemetry.debug("Start / Options: " + g1.options);
        //Telemetry.debug("Back / Share: " + g1.back);
        //Telemetry.debug("Guide / PS: " + g1.guide);
        //Telemetry.debug("Touchpad: " + g1.touchpad);
        //Telemetry.debug("Left Stick Button: " + g1.left_stick_button);
        //Telemetry.debug("Right Stick Button: " + g1.right_stick_button);

        //Telemetry.debug("==== Sticks ====");
        //Telemetry.debug("Left Stick X: " + g1.left_stick_x);
        //Telemetry.debug("Left Stick Y: " + g1.left_stick_y);
        //Telemetry.debug("Right Stick X: " + g1.right_stick_x);
        //Telemetry.debug("Right Stick Y: " + g1.right_stick_y);
        //Telemetry.update();
    }

}