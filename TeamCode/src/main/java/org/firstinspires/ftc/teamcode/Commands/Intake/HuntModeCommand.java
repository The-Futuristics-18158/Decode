package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class HuntModeCommand extends CommandBase {
    private boolean haveArtifact;
    private double blobX;
    PIDController omegaControl;
    //boolean finished;
    int finishedCounter;
    private ElapsedTime timer;
    private double seconds;

    // constructor
    public HuntModeCommand(){
        this(5.0);
    }
    public HuntModeCommand(double time) {
        haveArtifact = false;

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
        omegaControl = new PIDController(0.01, 0.0, 0.0);
        timer = new ElapsedTime();
        seconds = time;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        omegaControl.reset();
        //finished = false;
        finishedCounter=0;
        timer.reset();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        // gets list of blobs from the camera
        double x_speed;
        double y_speed;
        double omega_speed;

        if (timer.seconds() >= seconds || (RobotContainer.colour.isLeftArtifactPresent() &&
                RobotContainer.colour.isRightArtifactPresent()
                && RobotContainer.distance.isRampArtifactPresent())){

            RobotContainer.intake.intakeStop();
            finishedCounter++;
            //finished = true;

        }else {
            finishedCounter=0;
            RobotContainer.intake.intakeRun();
        }

        List<ColorBlobLocatorProcessor.Blob> blobs;
        // change this later if for get green blob detections and or get purple blob detections (if we want to look for one specific color)
        blobs = RobotContainer.rampCamera.GetAllBlobDetections();

        if (blobs != null && !(blobs.isEmpty())) {
            haveArtifact = true;
            blobX = blobs.get(0).getCircle().getCenter().x - 160.0;

            // set forward speed to value depending on how far artifact from center of camera
            x_speed = 0.6 - 0.5*Math.min(Math.abs(blobX/160.0),1.0);

            // determine sideways speed
            omega_speed = omegaControl.calculate(blobX); //320
        } else {
            haveArtifact = false;
            blobX = 0;
            omega_speed = 0.0;
            x_speed = 0.8;
        }

       // if busy intaking robot, stop and wait for it to finish before proceeding
        if (RobotContainer.distance.isRampArtifactPresent())
        //if (RobotContainer.artifactCamera.IsBottomPresent())
           x_speed = 0.0;
       //else
       //     x_speed = 0.5;//was 0.6 // was 0.5

        y_speed= 0;

        RobotContainer.drivesystem.RobotDrive(x_speed, y_speed, omega_speed);

    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished(){

        return (finishedCounter>=3);

        //return finished;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.drivesystem.RobotDrive(0,0,0);
        RobotContainer.intake.intakeStop();

    }

}