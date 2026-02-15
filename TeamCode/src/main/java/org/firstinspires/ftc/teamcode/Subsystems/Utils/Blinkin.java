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
    private boolean hasGreen = false;
    private boolean hasPurple = false;
    private  boolean hasArtifact = false;
    private boolean hasTag = false;
//    private boolean seesPurple;
//    private boolean seesGreen;
    private ElapsedTime timer;
    // Local objects and variables here

    /** Place code here to initialize subsystem */
    public Blinkin() {
        blinkin = RobotContainer.ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        timer = new ElapsedTime();
        timer.reset();
        Blink();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
       hasArtifact = (RobotContainer.artifactCamera.IsLeftPresent()|| RobotContainer.artifactCamera.IsRightPresent());
       hasGreen = (RobotContainer.artifactCamera.getLeftColour().equals(ArtifactCamera.ArtifactColours.Green)||
               RobotContainer.artifactCamera.getRightColour().equals(ArtifactCamera.ArtifactColours.Green ));
       hasPurple = (RobotContainer.artifactCamera.getLeftColour().equals(ArtifactCamera.ArtifactColours.Purple)||
               RobotContainer.artifactCamera.getRightColour().equals(ArtifactCamera.ArtifactColours.Purple));
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

    public void ShowPurple(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }

    public void ShowGreen(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }


    public void ShowGoal(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }
    public void Blink(){
      if (timer.seconds() % 1.5 == 0 && hasTag){
          if (hasTag){
              ShowGoal();
          }else {
              ShowAlliance();
          }
      }else if (timer.seconds() % 1.0 == 0){
          if (hasPurple){
              ShowPurple();
          }else{
              ShowAlliance();
          }
      }else if (timer.seconds() % 0.5 == 0){
         if (hasGreen){
              ShowGreen();
          }else{
              ShowAlliance();
          }
      }
    }


}