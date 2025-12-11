package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.GoalTargeting;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;
import org.firstinspires.ftc.teamcode.Utility.Utils;

public class AimToShoot extends CommandBase {
    private double TargetX;
    PIDController omegaControl;
    double TargetAngleOffset;
    ElapsedTime OnTargetTime;

    // our shooting solution lambdA
    GoalTargeting.LeftVsRight leftvsright;

    // constructor
    public AimToShoot() { this(null); }

    public AimToShoot(GoalTargeting.LeftVsRight solution) {
        this.leftvsright = solution;
        addRequirements(RobotContainer.drivesystem);
        //omegaControl = new PIDController(0.075, 0.1, 0.0);
        omegaControl = new PIDController(0.075, 0.60, 0.02);
        OnTargetTime = new ElapsedTime();
        TargetAngleOffset=0.0;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // assume 0deg offset unless determined otherwise
        TargetAngleOffset = 0.0;

        // were we supplied with a left vs right selection?
        if (leftvsright!=null) {
            // get the side from our target solution
            // and determine target offset to use
            GoalTargeting.ShootSide side = leftvsright.getSide();

            // determine target angle offset depending on which side we will shoot from
            if (side == GoalTargeting.ShootSide.LEFT)
                TargetAngleOffset = 0.0; // was -1.5
            if (side == GoalTargeting.ShootSide.RIGHT)
                TargetAngleOffset = 0.0; // was 1.5
        }

        omegaControl.reset();
        OnTargetTime.reset();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        // gets list of blobs from the camera
        double x_speed;
        double y_speed;
        double omega_speed;

        //Pose2d pose = RobotContainer.odometry.getCurrentPos();
        //Translation2d targetPose = AutoFunctions.redVsBlue(new Translation2d(-1.63, -1.63));
        //double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
        //TargetX = -Utils.AngleDifference(pose.getRotation().getDegrees(), Math.toDegrees(angle_rad));
        //omega_speed = omegaControl.calculate(TargetX);

        // get limelight targeting results
        LLResultTypes.FiducialResult target = RobotContainer.limeLight.getTargetInfo();


        double DistanceToTarget = RobotContainer.targeting.GetDistanceToGoal();
        //double Tolerance = 2.0 - 0.5*DistanceToTarget;
        DistanceToTarget = Math.max(0.3, DistanceToTarget);
        DistanceToTarget = Math.min(3.0, DistanceToTarget);
        double Tolerance = Math.toDegrees(Math.atan(0.02662/DistanceToTarget));



        //double Tolerance = 1.0;
        //if (RobotContainer.odometry.getCurrentPos().getX() > 0.6)
        //    Tolerance = 0.5;


        if (target != null) {
            // we have limelight target - determine rotational speed from pid
            TargetX = target.getTargetXDegrees() + TargetAngleOffset;

            // if angle too large, than reset integrated error
            if (Math.abs(TargetX) > 4.0){omegaControl.reset();}

            omega_speed = omegaControl.calculate(TargetX);
        }
        else {
            // we don't have limelight target - use odometry

            Pose2d pose = RobotContainer.odometry.getCurrentPos();
            Translation2d targetPose = AutoFunctions.redVsBlue(new Translation2d(-1.73, -1.73)); // was -1.63, -1.63

            //if (RobotContainer.odometry.getCurrentPos().getX() > 0.6){
            //    targetPose = AutoFunctions.redVsBlue(new Translation2d(-1.53, -1.63));
            //    //Tolerance = 0.5;
            //}

            double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
            TargetX = -Utils.AngleDifference(pose.getRotation().getDegrees(), Math.toDegrees(angle_rad)+ TargetAngleOffset);

            // if angle too large, than reset integrated error
            if (Math.abs(TargetX) > 4.0){omegaControl.reset();}

            omega_speed = omegaControl.calculate(TargetX);
        }



        // we are not moving in x or y direction so set both to 0
        x_speed = 0.0;
        y_speed= 0;

        RobotContainer.drivesystem.RobotDrive(x_speed, y_speed, omega_speed);

        // if we don't have target or not within 0.5deg, reset timer back to 0
        if (Math.abs(TargetX)>Tolerance)  // was 0.5 , was 3.0
            OnTargetTime.reset();
        //else
            //omegaControl.reset();

        double ilimit = 0.35*Math.abs(TargetX);
        ilimit = Math.max(ilimit, 0.35);
        omegaControl.setIntegrationBounds(-ilimit, ilimit);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished(){

        return OnTargetTime.seconds() >= 0.10;  // was 0.25
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivesystem.RobotDrive(0,0,0);
    }

}