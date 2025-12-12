package org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;



/**
 * Place description of subsystem here
 * leftFrontDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
 * @author Blackthrush
 */
public class ColourSensor extends SubsystemBase {

    // Local objects and variables here
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;
    //private ColorSensor rampSensor;
    private Rev2mDistanceSensor rampSensor;

    /** Place code here to initialize subsystem */
    public ColourSensor() {
    leftSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "leftColorSensor");
    rightSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "rightColorSensor");
    rampSensor = RobotContainer.ActiveOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "rampDistance");
    rampSensor.initialize();

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }
    // place special subsystem methods here
    public enum ArtifactColours{
        Purple,
        Green,
        Nothing
    }

    public boolean isLeftArtifactPresent() {
        try {
            return (leftSensor.alpha() > 70.0);

        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.RCTelemetry.addData("Left Sensor Error", e.getMessage());
            RobotContainer.RCTelemetry.update();
            try {
                leftSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "leftColorSensor");
            } catch (Exception ex) {
                // failed to recover
                RobotContainer.RCTelemetry.addData("Left Sensor recovery failed", ex.getMessage());
                RobotContainer.RCTelemetry.update();
            }

            return false;
        }
    }


    public ArtifactColours GetLeftColour(){
       if (isLeftArtifactPresent() == true){
           if(leftSensor.green()> leftSensor.blue()){
               return ArtifactColours.Green;
           }else{
               return ArtifactColours.Purple;
           }
       }else{
           return ArtifactColours.Nothing;
       }
    }

    public boolean isRightArtifactPresent(){

        try {
            return (rightSensor.alpha() >70.0);

        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.RCTelemetry.addData("Right Sensor Error", e.getMessage());
            RobotContainer.RCTelemetry.update();
            try {
                rightSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "rightColorSensor");
            } catch (Exception ex) {
                // failed to recover
                RobotContainer.RCTelemetry.addData("Right Sensor recovery failed", ex.getMessage());
                RobotContainer.RCTelemetry.update();
            }

            return false;
        }

    }
    public ArtifactColours GetRightColour(){
        if (isRightArtifactPresent() == true){
            if(rightSensor.green()> rightSensor.blue()){
                return ArtifactColours.Green;
            }else{
                return ArtifactColours.Purple;
            }
        }else{
            return ArtifactColours.Nothing;
        }
    }

    private boolean isRampSensorInitialized = true;

    public boolean isRampArtifactPresent(){
        try {
            double distance = rampSensor.getDistance(DistanceUnit.MM);
            return (distance >= 55.0 && distance <= 130.0);
        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.RCTelemetry.addData("rampDistance Sensor Error", e.getMessage());
            RobotContainer.RCTelemetry.update();
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
                    RobotContainer.RCTelemetry.addData("rampDistance Sensor recovery failed", ex.getMessage());
                    RobotContainer.RCTelemetry.update();
                }
            }
            return false; // Can't determine presence, assume no artifact
        }
    }


}