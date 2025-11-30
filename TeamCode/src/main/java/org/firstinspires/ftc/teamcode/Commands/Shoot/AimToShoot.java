package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.internal.opengl.models.Geometry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;
import org.firstinspires.ftc.teamcode.Utility.Utils;

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
        omegaControl = new PIDController(0.15, 0.0008, 0.0);
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

        Pose2d pose = RobotContainer.odometry.getCurrentPos();
        Translation2d targetPose = AutoFunctions.redVsBlue(new Translation2d(-1.63, -1.63));
        double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        TargetX = -Utils.AngleDifference(pose.getRotation().getDegrees(), Math.toDegrees(angle_rad));
        omega_speed = omegaControl.calculate(TargetX);

//        LLResultTypes.FiducialResult target = RobotContainer.limeLight.getTargetInfo();
//
//        if (target != null) {
//            haveTarget = true;
//            TargetX = target.getTargetXDegrees();
//
//            // determine sideways speed
//            omega_speed = omegaControl.calculate(TargetX); //320
//        } else {
//            haveTarget = false;
//            TargetX = 0;
//            omega_speed = 0.0;
//        }

        x_speed = 0.0;
        y_speed= 0;

        if (Math.abs(TargetX)<0.5){
            OnTargetTime += 0.02;
        }else{
            OnTargetTime = 0.0;
        }

        RobotContainer.drivesystem.RobotDrive(x_speed, y_speed, omega_speed);

    }
    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished(){

        return OnTargetTime >= 0.025;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0,0,0);
    }

}