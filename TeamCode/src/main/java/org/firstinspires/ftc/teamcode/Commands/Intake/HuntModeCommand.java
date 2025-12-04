package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class HuntModeCommand extends CommandBase {
    private boolean haveArtifact;
    private double blobX;
    PIDController omegaControl;
    boolean finished;
    
    // constructor
    public HuntModeCommand() {
        haveArtifact = false;
        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
        omegaControl = new PIDController(0.02, 0.0, 0.0);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        omegaControl.reset();
        finished = false;
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        // gets list of blobs from the camera
        double x_speed;
        double y_speed;
        double omega_speed;

        if ((RobotContainer.colour.isLeftArtifactPresent() && RobotContainer.colour.isRightArtifactPresent() && RobotContainer.colour.isRampArtifactPresent())){
            finished = true;
            RobotContainer.intake.intakeStop();
        }else {
            RobotContainer.intake.intakeRun();
        }

        List<ColorBlobLocatorProcessor.Blob> blobs;
        // change this later if for get green blob detections and or get purple blob detections (if we want to look for one specific color)
        blobs = RobotContainer.rampCamera.GetAllBlobDetections();

        if (blobs != null && !(blobs.isEmpty())) {
            haveArtifact = true;
            blobX = blobs.get(0).getCircle().getCenter().x - 160.0;

            // determine sideways speed
            omega_speed = omegaControl.calculate(blobX); //320
        } else {
            haveArtifact = false;
            blobX = 0;
            omega_speed = 0.0;
        }

       // if busy intaking robot, stop and wait for it to finish before proceeding
        if (RobotContainer.colour.isRampArtifactPresent())
           x_speed = 0.0;
       else
            x_speed = 0.5;//was 0.6

        y_speed= 0;

        RobotContainer.drivesystem.RobotDrive(x_speed, y_speed, omega_speed);

    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished(){

        return finished;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.drivesystem.RobotDrive(0,0,0);
        RobotContainer.intake.intakeStop();

    }

}