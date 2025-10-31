package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
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

    // state values to determine speed and acceleration
    private ElapsedTime dt;
    private Pose2d previousPose;
    private Pose2d currentPose;
    private Speed previousSpeed;
    private Speed filteredCurrentSpeed;  // double-order low pass filter 1st-stage
    private Speed filteredCurrentSpeed2; // double-order low pass filter 2nd-stage
    private Acceleration currentAcceleration;

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

        // reset state values
        dt = new ElapsedTime();
        dt.reset();
        previousPose = odometry.getPoseMeters();
        currentPose = previousPose;
        previousSpeed = new Speed();
        filteredCurrentSpeed = new Speed();
        filteredCurrentSpeed2 = new Speed();
        currentAcceleration = new Acceleration();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // get drive wheel speeds
        MecanumDriveWheelSpeeds speeds = RobotContainer.drivesystem.GetWheelSpeeds();

        // Correct FTCLib scaling error by multiplying all speeds by 1.0/sqrt(2)
        // Oct 29,2025 KN
        speeds.frontLeftMetersPerSecond  *= 0.7071068;
        speeds.frontRightMetersPerSecond *= 0.7071068;
        speeds.rearLeftMetersPerSecond   *= 0.7071068;
        speeds.rearRightMetersPerSecond  *= 0.7071068;

        // update odometry based on drive wheel speeds
        odometry.updateWithTime(time.seconds(),
                new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle())),
                speeds);

        // get time since last iteration (in seconds)
        // once getting time, reset timer for next iteration
        double t=dt.seconds();
        dt.reset();

        if (t!=0.0) {
            // inverse of time-slice
            double inv_t = 1.0 / t;

            // get current robot pose (m)
            currentPose = odometry.getPoseMeters();

            // determine current field speed (raw unfiltered) v=ds/dt
            Speed rawSpeed = new Speed();
            rawSpeed.vx = (currentPose.getX() - previousPose.getX()) * inv_t;
            rawSpeed.vy = (currentPose.getY() - previousPose.getY()) * inv_t;
            rawSpeed.omega = (currentPose.getRotation().getRadians() - previousPose.getRotation().getRadians()) * inv_t;

            // low pass filter the speeds (1st stage filter)
            filteredCurrentSpeed.vx = 0.85*filteredCurrentSpeed.vx + 0.15*rawSpeed.vx;
            filteredCurrentSpeed.vy = 0.85*filteredCurrentSpeed.vy + 0.15*rawSpeed.vy;
            filteredCurrentSpeed.omega = 0.85*filteredCurrentSpeed.omega + 0.15*rawSpeed.omega;
            // low pass filter the speeds (2nd stage filter)
            filteredCurrentSpeed2.vx = 0.85*filteredCurrentSpeed2.vx + 0.15*filteredCurrentSpeed.vx;
            filteredCurrentSpeed2.vy = 0.85*filteredCurrentSpeed2.vy + 0.15*filteredCurrentSpeed.vy;
            filteredCurrentSpeed2.omega = 0.85*filteredCurrentSpeed2.omega + 0.15*filteredCurrentSpeed.omega;
            // note: filteredCurrentSpeed2 now represents current filtered speed

            // determine current acceleration a=dv/dt
            currentAcceleration.ax = (filteredCurrentSpeed2.vx - previousSpeed.vx) * inv_t;
            currentAcceleration.ay = (filteredCurrentSpeed2.vy - previousSpeed.vy) * inv_t;
            currentAcceleration.alpha = (filteredCurrentSpeed2.omega - previousSpeed.omega) * inv_t;

            // Update states for the next call
            // current position and filtered speed becomes previous for next iteration
            previousPose = currentPose;
            previousSpeed.vx = filteredCurrentSpeed2.vx;
            previousSpeed.vy = filteredCurrentSpeed2.vy;
            previousSpeed.omega = filteredCurrentSpeed2.omega;

            // update filtered speed state variable
            filteredCurrentSpeed2.vx = filteredCurrentSpeed.vx;
            filteredCurrentSpeed2.vy = filteredCurrentSpeed.vy;
            filteredCurrentSpeed2.omega = filteredCurrentSpeed.omega;

            //RobotContainer.Panels.Telemetry.addData("wheel vx", filteredCurrentSpeed2.vx);
            //RobotContainer.Panels.Telemetry.addData("wheel vy", filteredCurrentSpeed2.vy);
            //RobotContainer.Panels.Telemetry.addData("wheel ax", currentAcceleration.ax);
            //RobotContainer.Panels.Telemetry.addData("wheel ay", currentAcceleration.ay);
            //RobotContainer.Panels.Telemetry.addData("wheel Chassis-vx", GetChassisSpeed().vx);
            //RobotContainer.Panels.Telemetry.addData("wheel Chassis-ax", GetChassisAcceleration().ax);
            //RobotContainer.Panels.Telemetry.update();

        } // end if (t!=0.0)

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

    /**
     * Returns robot field speed based on drive wheel odometry
     *
     * @return the robot x,y,angular field speeds (in m/s and rad/s)
     */
    public class Speed {public double vx=0.0; public double vy=0.0; public double omega=0.0; }
    public Speed GetSpeed() {
        return filteredCurrentSpeed2;
    }

    /**
     * Returns robot chassis speed based on drive wheel odometry
     *
     * @return the robot x,y,angular chassis speeds (in m/s and rad/s)
     */
    public Speed GetChassisSpeed() {
        // to get chassis speeds, must rotate field vector
        Speed speed = new Speed();
        double angle = Math.toRadians(RobotContainer.gyro.getYawAngle());
        speed.vx = filteredCurrentSpeed2.vx * Math.cos(-angle) - filteredCurrentSpeed2.vy * Math.sin(-angle);
        speed.vy = filteredCurrentSpeed2.vx * Math.sin(-angle) + filteredCurrentSpeed2.vy * Math.cos(-angle);
        speed.omega = filteredCurrentSpeed2.omega;
        return speed;
    }

    /**
     * Returns robot speed based on drive wheel odometry
     *
     * @return the robot x,y,angular field accelerations (in m/s2 and rad/s2)
     */
    public class Acceleration { double ax=0.0; double ay=0.0; double alpha=0.0;}
    public Acceleration GetAcceleration() {
        return currentAcceleration;
    }

    /**
     * Returns robot chassis acceleration based on drive wheel odometry
     *
     * @return the robot x,y,angular chassis acceleration (in m/s2 and rad/s2)
     */
    public Acceleration GetChassisAcceleration() {
        // to get chassis acceleration, must rotate field vector
        Acceleration accel = new Acceleration();
        double angle = Math.toRadians(RobotContainer.gyro.getYawAngle());
        accel.ax = currentAcceleration.ax * Math.cos(-angle) - currentAcceleration.ay * Math.sin(-angle);
        accel.ay = currentAcceleration.ax * Math.sin(-angle) + currentAcceleration.ay * Math.cos(-angle);
        accel.alpha = currentAcceleration.alpha;
        return accel;
    }


}