package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class AimToShoot extends CommandBase {
    private boolean haveTarget;
    private double TargetX;
    PIDController omegaControl;
    double OnTargetTime;

    // constructor
    public AimToShoot() {
        haveTarget = false;
        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
        omegaControl = new PIDController(0.01, 0.0005, 0.0);
        OnTargetTime = 0;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        omegaControl.reset();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        // gets list of blobs from the camera
        double x_speed;
        double y_speed;
        double omega_speed;

        LLResultTypes.FiducialResult target = RobotContainer.limeLight.getTargetInfo();


        if (target != null) {
            haveTarget = true;
            TargetX = target.getTargetXDegrees();

            // determine sideways speed
            omega_speed = omegaControl.calculate(TargetX); //320
        } else {
            haveTarget = false;
            TargetX = 0;
            omega_speed = 0.0;
        }

        x_speed = 0.0;
        y_speed= 0;

        if (haveTarget == true && Math.abs(TargetX)<0.5){
            OnTargetTime += 0.02;
        }else{
            OnTargetTime = 0.0;
        }

        RobotContainer.drivesystem.RobotDrive(x_speed, y_speed, omega_speed);

    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished(){

        return OnTargetTime >= 0.25;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0,0,0);
    }

}