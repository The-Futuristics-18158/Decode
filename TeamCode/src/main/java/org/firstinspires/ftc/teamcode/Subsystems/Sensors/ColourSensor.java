package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author Blackthrush
 */
public class ColourSensor extends SubsystemBase {

    // Local objects and variables here
    private ColorRangeSensor leftSensor;
    private ColorRangeSensor rightSensor;

    /**
     * Place code here to initialize subsystem
     */
    public ColourSensor() {
        leftSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorRangeSensor.class, "leftColorSensor");
        rightSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorRangeSensor.class, "rightColorSensor");
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */

    public boolean isLeftArtifactPresent() {

        // if left sensor is not working, ask camera (right side)
        if (!isLeftSensorAlive())
            return RobotContainer.artifactCamera.IsRightPresent();

        // otherwise, if working then ask sensor
        else {

            try {
                return (leftSensor.getDistance(DistanceUnit.MM) < 55.0);

            } catch (Exception e) {
                // Log the error (if you can), and handle recovery
                RobotContainer.telemetrySubsystem.addData("Left Sensor Error", e.getMessage(), true);
                RobotContainer.telemetrySubsystem.update();
                try {
                    leftSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorRangeSensor.class, "leftColorSensor");
                } catch (Exception ex) {
                    // failed to recover
                    RobotContainer.telemetrySubsystem.addData("Left Sensor recovery failed", ex.getMessage(), true);
                    RobotContainer.telemetrySubsystem.update();
                }
                return false;
            }
        }
    }

    public boolean isRightArtifactPresent(){

        // if right sensor is not working, ask camera (left side)
        if (!isRightSensorAlive())
            return RobotContainer.artifactCamera.IsLeftPresent();

            // otherwise, if working then ask sensor
        else {

            try {
                return (rightSensor.getDistance(DistanceUnit.MM) < 55.0);

            } catch (Exception e) {
                // Log the error (if you can), and handle recovery
                RobotContainer.telemetrySubsystem.addData("Right Sensor Error", e.getMessage(), true);
                RobotContainer.telemetrySubsystem.update();
                try {
                    rightSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorRangeSensor.class, "rightColorSensor");
                } catch (Exception ex) {
                    // failed to recover
                    RobotContainer.telemetrySubsystem.addData("Right Sensor recovery failed", ex.getMessage(), true);
                    RobotContainer.telemetrySubsystem.update();
                }
                return false;
            }
        }
    }


    public boolean isLeftSensorAlive() {
        try {
            return (leftSensor.getDistance(DistanceUnit.MM) < 1000.0);

        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.telemetrySubsystem.addData("Left Sensor Error", e.getMessage(), true);
            RobotContainer.telemetrySubsystem.update();
            try {
                leftSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorRangeSensor.class, "leftColorSensor");
            } catch (Exception ex) {
                // failed to recover
                RobotContainer.telemetrySubsystem.addData("Left Sensor recovery failed", ex.getMessage(), true);
                RobotContainer.telemetrySubsystem.update();
            }
            return false;
        }
    }

    public boolean isRightSensorAlive(){

        try {
            return (rightSensor.getDistance(DistanceUnit.MM) < 1000.0);

        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.telemetrySubsystem.addData("Right Sensor Error", e.getMessage(), true);
            RobotContainer.telemetrySubsystem.update();
            try {
                rightSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorRangeSensor.class, "rightColorSensor");
            } catch (Exception ex) {
                // failed to recover
                RobotContainer.telemetrySubsystem.addData("Right Sensor recovery failed", ex.getMessage(), true);
                RobotContainer.telemetrySubsystem.update();
            }
            return false;
        }
    }




}