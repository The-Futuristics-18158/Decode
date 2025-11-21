package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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

    /** Place code here to initialize subsystem */
    public ColourSensor() {
    leftSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "leftColorSensor");
    rightSensor = RobotContainer.ActiveOpMode.hardwareMap.get(ColorSensor.class, "rightColorSensor");

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

    public boolean isLeftArtifactPresent(){
        if (leftSensor.alpha() > 80.0){
            return true;
        }else{
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
        if (rightSensor.alpha() > 80.0){
            return true;
        }else{
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
    // what will appear on the driverstation
    // Left Alphaness: 93
    // left redness: 74
    // Left Greeness: 91
    //Left Blueness: 120

    // Right Alphaness: 95
    //Right Redness: 49
    //Right Greeness : 135
    // Right Bluesness: 106

}