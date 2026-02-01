package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Cameras.ArtifactCamera;

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
        if(RobotContainer.artifactCamera.IsLeftPresent()|| RobotContainer.artifactCamera.IsRightPresent()) {
            ShowBallColours();
            //if (RobotContainer.limeLight.hasGoal){
            //    ShowHasGoal();
            //}
//        }else if(RobotContainer.climb.climbStop()){
//            ShowHasClimbed();

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

//    public void ShowHasClimbed(){
//        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
//    }

    public void ShowBallColours(){
       if(RobotContainer.artifactCamera.getLeftColour().equals(ArtifactCamera.ArtifactColours.Green)||
               RobotContainer.artifactCamera.getRightColour().equals(ArtifactCamera.ArtifactColours.Green )){
           hasGreen = true;
       }
       else
           hasGreen = false;

       if(RobotContainer.artifactCamera.getLeftColour().equals(ArtifactCamera.ArtifactColours.Purple)||
               RobotContainer.artifactCamera.getRightColour().equals(ArtifactCamera.ArtifactColours.Purple ) ){
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