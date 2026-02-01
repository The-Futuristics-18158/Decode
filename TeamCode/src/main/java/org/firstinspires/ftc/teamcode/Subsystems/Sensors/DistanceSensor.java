package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * Place description of subsystem here
 *
 * @author Blackthrush
 */
public class DistanceSensor extends SubsystemBase {

    // Local objects and variables here
    private Rev2mDistanceSensor rampSensor;

    /** Place code here to initialize subsystem */
    public DistanceSensor() {

    rampSensor = RobotContainer.ActiveOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "rampDistance");
    rampSensor.initialize();

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {


    }

    private boolean isRampSensorInitialized = true;

    public boolean isRampArtifactPresent(){
        try {
            double distance = rampSensor.getDistance(DistanceUnit.MM);
            return (distance >= 55.0 && distance <= 130.0);
        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.telemetrySubsystem.addData("rampDistance Sensor Error", e.getMessage());
            RobotContainer.telemetrySubsystem.update();
            // Try to recover the sensor
            if (isRampSensorInitialized) {
                // First error: attempt to re-initialize
                isRampSensorInitialized = false;
                try {
                    // Re-fetch and re-initialize sensor
                    rampSensor = RobotContainer.ActiveOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "rampDistance");
                    rampSensor.initialize();
                    isRampSensorInitialized = true;
                } catch (Exception ex) {
                    // failed to recover
                    RobotContainer.telemetrySubsystem.addData("rampDistance Sensor recovery failed", ex.getMessage());
                    RobotContainer.telemetrySubsystem.update();
                }

            }
            return false; // Can't determine presence, assume no artifact
        }
    }


}