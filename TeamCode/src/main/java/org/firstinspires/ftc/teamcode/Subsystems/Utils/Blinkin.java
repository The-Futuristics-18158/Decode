package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Cameras.ArtifactCamera;
import org.firstinspires.ftc.teamcode.Subsystems.Cameras.RampCamera;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

/**
 * Place description of subsystem here
 *
 * @author kaitlyn
 */
public class Blinkin extends SubsystemBase {
    private RevBlinkinLedDriver blinkin;
    private boolean hasGreen;
    private boolean hasPurple;
//    private boolean seesPurple;
//    private boolean seesGreen;
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
        if(RobotContainer.artifactCamera.IsLeftPresent()|| RobotContainer.artifactCamera.IsRightPresent()) {
            ShowBallColours();

        } else{
           ShowAlliance();
       }
    }

    // place special subsystem methods here

    /**
     * shows alliance color on the blinkin - red for red alliance, blue for blue alliance
     */
    public void ShowAlliance(){
        if (RobotContainer.isRedAlliance()){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else{
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }

    /**
     * shows ball colors on the blinkin - green for green ball, purple for purple ball
     */
    public void ShowBallColours(){

//        // Blobs list
//        List<ColorBlobLocatorProcessor.Blob> blobs;
//
//     -------------------------- Sees Green --------------------------
//        // Setting Green Detection Variable
//        blobs = RobotContainer.rampCamera.GetGreenBlobDetections();
//        // Sees Green
//        if (blobs != null && !(blobs.isEmpty())){
//            seesGreen = true;
//        } else {
//            seesGreen = false;
//        }
//    -------------------------- Sees Purple --------------------------
//        // Setting Purple Detection Variable
//        blobs = RobotContainer.rampCamera.GetPurpleBlobDetections();
//        // Sees Purple
//        if (blobs != null && !(blobs.isEmpty())){
//            seesPurple = true;
//        } else {
//            seesPurple = false;
//        }
//     -------------------------- Sees Purple and Green --------------------------
//        if (seesPurple && seesGreen){
//            if (timer.seconds() <=0.5){
//                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
//
//            } else if (timer.seconds() <=1){
//                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
//
//            } else {
//                timer.reset();
//            }
//        }
//      -------------------------- Has Green --------------------------
        if(RobotContainer.artifactCamera.getLeftColour().equals(ArtifactCamera.ArtifactColours.Green)||
               RobotContainer.artifactCamera.getRightColour().equals(ArtifactCamera.ArtifactColours.Green ))
        {
           hasGreen = true;
        } else {
           hasGreen = false;
        }
//      -------------------------- Has Purple --------------------------
        if(RobotContainer.artifactCamera.getLeftColour().equals(ArtifactCamera.ArtifactColours.Purple)||
               RobotContainer.artifactCamera.getRightColour().equals(ArtifactCamera.ArtifactColours.Purple))
        {
            hasPurple = true;

//      -------------------------- Has Purple and Green --------------------------
            if (hasPurple && hasGreen){
                if (timer.seconds() <=0.5){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                } else if (timer.seconds() <=1){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

                } else if (timer.seconds() <=1.5) {
                    if (RobotContainer.limeLight.hasGoal) {
                        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                    }
                } else{
                    timer.reset();
                }
            }
//      -------------------------- Setting Blinkin Colors --------------------------
       } else if (hasPurple){
           blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
       } else if (hasGreen){
           blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//       } else if (seesPurple) {
//           blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
//       } else if (seesGreen) {
//           blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
       }
    }
}