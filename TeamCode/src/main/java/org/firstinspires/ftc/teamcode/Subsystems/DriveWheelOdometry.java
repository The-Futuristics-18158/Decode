package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * Temporary subsystem that tracks robot odometry using drive wheel rotations.
 * Subsystem used to compare with other odometry to determine wheel slippage
 *
 * @author knutt5
 *
 */
public class DriveWheelOdometry extends SubsystemBase {

    // drive wheel odometry
    private MecanumDriveOdometry odometry;

    // timer to use with odometry
    private ElapsedTime time;

    /**
     * Constuctor
     */
    public DriveWheelOdometry() {

        // get mecanum kinematics from drive
        MecanumDriveKinematics kinematics = RobotContainer.drivesystem.GetKinematics();

        // get gyro angle
        Rotation2d GyroAngle = new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle()));

        // create drive wheel odometry
        odometry = new MecanumDriveOdometry(kinematics, GyroAngle);

        // initialize timer
        time = new ElapsedTime();
        time.reset();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // update odometry based on drive wheel speeds
        odometry.updateWithTime(time.seconds(),
                                new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle())),
                                RobotContainer.drivesystem.GetWheelSpeeds());
    }

    /**
     * Sets odometry to the given position
     *
     * @param position the position to set odometry to (Pose2d) in meters
     */
    public void SetPosition(Pose2d position) {
        odometry.resetPosition(position, new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle())));
    }


    /**
     * Returns current position of robot based on drive wheel odometry
     *
     * @return the robot odometry Posd2d in meters
    */
    public Pose2d GetPosition() {
        return odometry.getPoseMeters();
    }

}

