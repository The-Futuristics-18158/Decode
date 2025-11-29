package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class AimToShoot extends CommandBase {
    private boolean haveArtifact;
    private double blobX;
    PIDController omegaControl;

    // constructor
    public AimToShoot() {
        haveArtifact = false;
        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
        omegaControl = new PIDController(0.015, 0.0, 0.0);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        omegaControl.reset();
        RobotContainer.intake.intakeRun();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        // gets list of blobs from the camera
        double x_speed;
        double y_speed;
        double omega_speed;

        List<ColorBlobLocatorProcessor.Blob> blobs;
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

        x_speed = 0.9;//was 0.20
        y_speed= 0;

        RobotContainer.drivesystem.RobotDrive(x_speed, y_speed, omega_speed);

    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished(){

        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.drivesystem.RobotDrive(0,0,0);
        RobotContainer.intake.intakeStop();

    }

}