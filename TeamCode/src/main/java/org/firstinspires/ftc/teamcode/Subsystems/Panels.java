package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author Kw126
 */
public class Panels extends SubsystemBase {

    // Local objects and variables here
    com.bylazar.battery.PanelsBattery battery;
    com.bylazar.gamepad.PanelsGamepad gamepad;
    com.bylazar.telemetry.PanelsTelemetry panelsTelemetry;




    /** Place code here to initialize subsystem */
    public Panels() {

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here
    public double readBattery(){
        return battery.getProvider().getBatteryVoltage();
    };

    private void gamepadTelemetry(){
        // Get current gamepads from the active FTC hardware instance
        com.qualcomm.robotcore.hardware.Gamepad g1 = RobotContainer.driverOp.gamepad;
        com.qualcomm.robotcore.hardware.Gamepad g2 = RobotContainer.toolOp.gamepad;

        if (g1 == null || g2 == null) {
            panelsTelemetry.getTelemetry().debug("Gamepads not initialized yet!");
            panelsTelemetry.getTelemetry().update(telemetry);
            return;
        }


        panelsTelemetry.getTelemetry().debug("==== Buttons ====");
        panelsTelemetry.getTelemetry().debug("A: " + g1.a);
        panelsTelemetry.getTelemetry().debug("B: " + g1.b);
        panelsTelemetry.getTelemetry().debug("X: " + g1.x);
        panelsTelemetry.getTelemetry().debug("Y: " + g1.y);
        panelsTelemetry.getTelemetry().debug("DPad Up: " + g1.dpad_up);
        panelsTelemetry.getTelemetry().debug("DPad Down: " + g1.dpad_down);
        panelsTelemetry.getTelemetry().debug("DPad Left: " + g1.dpad_left);
        panelsTelemetry.getTelemetry().debug("DPad Right: " + g1.dpad_right);
        panelsTelemetry.getTelemetry().debug("Left Bumper: " + g1.left_bumper);
        panelsTelemetry.getTelemetry().debug("Right Bumper: " + g1.right_bumper);
        panelsTelemetry.getTelemetry().debug("Left Trigger: " + g1.left_trigger);
        panelsTelemetry.getTelemetry().debug("Right Trigger: " + g1.right_trigger);
        panelsTelemetry.getTelemetry().debug("Start / Options: " + g1.options);
        panelsTelemetry.getTelemetry().debug("Back / Share: " + g1.back);
        panelsTelemetry.getTelemetry().debug("Guide / PS: " + g1.guide);
        panelsTelemetry.getTelemetry().debug("Touchpad: " + g1.touchpad);
        panelsTelemetry.getTelemetry().debug("Left Stick Button: " + g1.left_stick_button);
        panelsTelemetry.getTelemetry().debug("Right Stick Button: " + g1.right_stick_button);

        panelsTelemetry.getTelemetry().debug("==== Sticks ====");
        panelsTelemetry.getTelemetry().debug("Left Stick X: " + g1.left_stick_x);
        panelsTelemetry.getTelemetry().debug("Left Stick Y: " + g1.left_stick_y);
        panelsTelemetry.getTelemetry().debug("Right Stick X: " + g1.right_stick_x);
        panelsTelemetry.getTelemetry().debug("Right Stick Y: " + g1.right_stick_y);

        panelsTelemetry.getTelemetry().update(telemetry);
    }
}