package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.ColourSensor;

/**
 * Place description of subsystem here
 *
 * @author Kw126
 */
public class Blinkin extends SubsystemBase {
    private RevBlinkinLedDriver blinkin;
    private boolean hasGreen;
    private boolean hasPurple;
    private ElapsedTime timer;
    // Local objects and variables here

    /** Place code here to initialize subsystem */
    public Blinkin() {
        blinkin = RobotContainer.ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        timer = new ElapsedTime();
        timer.reset();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        if(RobotContainer.colour.isLeftArtifactPresent()|| RobotContainer.colour.isRightArtifactPresent()){
            ShowBallColours();
            //if (RobotContainer.limeLight.hasGoal){
            //    ShowHasGoal();
            //}
        } else{
           ShowAlliance();
       }

    }

    // place special subsystem methods here
    public void ShowAlliance(){
        if (RobotContainer.isRedAlliance()){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else{
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }


    public void ShowHasGoal(){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void ShowBallColours(){
       if(RobotContainer.colour.GetLeftColour().equals(ColourSensor.ArtifactColours.Green)||
               RobotContainer.colour.GetRightColour().equals(ColourSensor.ArtifactColours.Green) ){
           hasGreen = true;
       }
       else
           hasGreen = false;

       if(RobotContainer.colour.GetLeftColour().equals(ColourSensor.ArtifactColours.Purple)||
                RobotContainer.colour.GetRightColour().equals(ColourSensor.ArtifactColours.Purple) ){
            hasPurple = true;
       }
       else
           hasPurple = false;

       if (hasPurple && hasGreen){
           if (timer.seconds() <=0.5){
               blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
           }
           else if (timer.seconds() <=1){
               blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
           }
           else if (timer.seconds() <=1.5) {
               if (RobotContainer.limeLight.hasGoal) {
                   blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
               }
           }
           else{
               timer.reset();
           }

       }else if (hasPurple){
           blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
       }else if (hasGreen){
           blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
       }
    }
}