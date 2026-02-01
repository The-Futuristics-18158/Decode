package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RobotContainer;



/**
 * Place description of subsystem here
 *
 * @author Blackthrush
 */
public class ColourSensor extends SubsystemBase {

    // Local objects and variables here
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;


    /**
     * Place code here to initialize subsystem
     */
    public ColourSensor() {
        leftSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "rightColorSensor");

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
//    @Override
//    public void periodic() {
//
//    }
    // place special subsystem methods here
//    public enum ArtifactColours{
//        Purple,
//        Green,
    //Nothing
//    }

    public boolean isLeftArtifactPresent() {
        try {
            return (leftSensor.alpha() > 80.0);

        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.telemetrySubsystem.addData("Left Sensor Error", e.getMessage(), true);
            RobotContainer.telemetrySubsystem.update();
            try {
                leftSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "leftColorSensor");
            } catch (Exception ex) {
                // failed to recover
                RobotContainer.telemetrySubsystem.addData("Left Sensor recovery failed", ex.getMessage(), true);
                RobotContainer.telemetrySubsystem.update();
            }

            return false;
        }
    }


//    public ArtifactColours GetLeftColour(){
//       if (isLeftArtifactPresent() == true){
//           if(leftSensor.green() > leftSensor.blue()){
//               return ArtifactColours.Green;
//           }else{
//               return ArtifactColours.Purple;
//           }
//       }else{
//           return ArtifactColours.Nothing;
//       }
//    }

    public boolean isRightArtifactPresent(){

        try {
            return (rightSensor.alpha() > 80.0);

        } catch (Exception e) {
            // Log the error (if you can), and handle recovery
            RobotContainer.telemetrySubsystem.addData("Right Sensor Error", e.getMessage(), true);
            RobotContainer.telemetrySubsystem.update();
            try {
                rightSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "rightColorSensor");
            } catch (Exception ex) {
                // failed to recover
                RobotContainer.telemetrySubsystem.addData("Right Sensor recovery failed", ex.getMessage(), true);
                RobotContainer.telemetrySubsystem.update();
            }

            return false;
        }

    }
//    public ArtifactColours GetRightColour(){
//        if (isRightArtifactPresent() == true){
//            if(rightSensor.green()> rightSensor.blue()){
//                return ArtifactColours.Green;
//            }else{
//                return ArtifactColours.Purple;
//            }
//        }else{
//            return ArtifactColours.Nothing;
//        }
//    }
}