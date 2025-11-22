package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
// EXAMPLE - use @Configurable to add public variables to dashboard for realtime updating
public class Odometry extends SubsystemBase {

    // stored robot position (static) used to keep robot pose between opmodes
    // value persists even if new odometry system is created.
    private static Pose2d StoredRobotPose = new Pose2d(0,0, new Rotation2d(0));

    // current robot position (in m)
    private Pose2d CurrentPose;

    /** Place code here to initialize subsystem */
    public Odometry() {

        // initialize field position from stored value (i.e. previous op-mode)
        setCurrentPos(StoredRobotPose);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // get/save current robot position
        RobotContainer.odometryPod.GetPose();

        // process any limelight or other odometry stuff here
        //
        // TBD

        // save position to data store, in case op mode ends
        // check again if op mode not about to shut down - otherwise don't save it
        if (!RobotContainer.ActiveOpMode.isStopRequested())
            StoredRobotPose = new Pose2d (CurrentPose.getX(), CurrentPose.getY(), CurrentPose.getRotation());

    }

    // place special subsystem methods here

    /**
     * returns the current robot pose
     * @return Pose2d of robot position (in m)
     */
    public Pose2d getCurrentPos() {
       return new Pose2d (CurrentPose.getX(), CurrentPose.getY(), CurrentPose.getRotation());
    }

    /**
     * sets robot position to provided Pose2d
     * @param pos - Pose2d of robot position (in m)
     */
    // sets robot position to provided Pose2d
    public void setCurrentPos(Pose2d pos){
        // record new robot position
        CurrentPose = pos;

        // set position of odometry pods
        RobotContainer.odometryPod.SetPose(pos);

        // set angle of gyro
        RobotContainer.gyro.setYawAngle(Math.toDegrees(pos.getHeading()));
    }

    /**
     * resets current pose2d to default (0,0,0deg)
     */
    public void resetCurrentPos(){
        setCurrentPos(new Pose2d(0,0,new Rotation2d(0)));
    }

}