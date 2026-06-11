package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * Place description of subsystem here
 *
 * @author Blackthrush
 */
public class UptakeSensor extends SubsystemBase {

    // Local objects and variables here
    private RevColorSensorV3 uptakeSensor;

    /** Place code here to initialize subsystem */
    public UptakeSensor() {

    uptakeSensor = RobotContainer.ActiveOpMode.hardwareMap.get(RevColorSensorV3.class, "uptakeSensor");
    uptakeSensor.initialize();

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {


    }

    private boolean isUptakeSensorInitialized = true;


    /**Uses the distance sensor to determine if an artifact is present on the ramp.
     * @return true if an artifact is detected, false otherwise
     */
    public boolean isUpakeArtifactPresent(){
        try {
            double distance = uptakeSensor.getDistance(DistanceUnit.CM);
            return (distance < 5.0);
        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.telemetrySubsystem.addData("uptake sensor Error", e.getMessage());
            RobotContainer.telemetrySubsystem.update();
            // Try to recover the sensor
            if (isUptakeSensorInitialized) {
                // First error: attempt to re-initialize
                isUptakeSensorInitialized = false;
                try {
                    // Re-fetch and re-initialize sensor
                    uptakeSensor = RobotContainer.ActiveOpMode.hardwareMap.get(RevColorSensorV3.class, "uptakeSensor");
                    uptakeSensor.initialize();
                    isUptakeSensorInitialized = true;
                } catch (Exception ex) {
                    // failed to recover
                    RobotContainer.telemetrySubsystem.addData("uptake sensor recovery failed", ex.getMessage());
                    RobotContainer.telemetrySubsystem.update();
                }

            }
            return false; // Can't determine presence, assume no artifact
        }
    }


}