package org.firstinspires.ftc.teamcode.Utility;
import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;

import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.DriveWheelOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;

/**
 * class that contains an automatic function that determines max acceleration that utilizes a while loop
 *
 * @author kw126
 * */
public class TestSlip {
    public static int writeDataIn = 25;
    public static int linesWritten = 0;
    private static Pose2d previousSlip;
    private static boolean isFirstDerivativeCalculation = true;
    public static final String FILE_NAME = AppUtil.getInstance().getWallClockTime() + "test slip file";

    // Variables for acceleration calculation
    private static Pose2d previousPose = new Pose2d();
    private static Twist2d previousVelocity = new Twist2d();
    private static long previousTime = System.nanoTime();
    private static Pose2d spreviousPose = new Pose2d();
    private static Twist2d spreviousVelocity = new Twist2d();
    private static long spreviousTime = System.nanoTime();

    public static double ax;
    public static double ay;
    public static double slipX;
    public static double slipY;

    /**
     * Function that can be used to determine the fastest acceleration the robot can use without creating slip
     *
     * @param maxSpeed      the max speed the robot is allowed to reach during the test movement
     * @param accToTest     the starting acceleration to test
     * @param distance      the distance the robot will travel during each test movement
     * @param maxBeforeTurn the max number of test movements before the robot direction flips
     * @param allowedError  an acceptable amount of slip
     * @return the fasted acceleration the robot can use without creating slip
     */
    public static int findMaxAcc(int maxSpeed, int accToTest, int distance, int maxBeforeTurn, double allowedError) {

        //declare variables
        int add = 10; // how much to add to the test acceleration. if the acceleration is too fast the amount to add will be changed.
        boolean tooFast = false; //was last tested too fast
        int turnIn = maxBeforeTurn - 1;
        boolean goForward = true;
        int lastTested = 0;//the last acceleration that was tested
        int lastOver = 0;//the last acceleration that was too fast
        double AverageSlip;
        Pose2d driveWheelPos; //the position from the drive wheel encoders
        Pose2d odometryPos;
        Pose2d slip;

        int direction;

        if (RobotContainer.isRedAlliance()) {
            direction = distance;
        } else {
            direction = -distance;
        }

        //WHILE ()
        while (true) {
            //code to make sure robot does not drive off mat
            if (turnIn == 0) {
                direction = direction * -1;
                turnIn = maxBeforeTurn;
            }

            // create command
            MoveToPose accelMove = new MoveToPose(maxSpeed, accToTest, RobotContainer.odometry.getCurrentPos().plus(new Transform2d(new Translation2d(0, direction), new Rotation2d())));

            // schedule command (add to scheduler)
            accelMove.schedule();


            // wait for path command to finish
            while (accelMove.isScheduled()) {
                CommandScheduler.getInstance().run();
                //RobotContainer.Periodic();
            }
            new WaitUntilCommand(accelMove::isFinished);

            // do some stuff to compare drive wheel odometry with pod odometry
            //compare odometry to determine if the acceleration is too fast
            slip = getAccumulatedSlip(true);
            AverageSlip = (Math.abs(slip.getX()) + Math.abs(slip.getY())) / distance;
            if (AverageSlip > allowedError) {
                tooFast = true;
            } else {
                tooFast = false;
            }

            // figure out some new parameters
            if (tooFast) {
                add = (accToTest - lastTested) / 2;
                lastOver = accToTest;
            } else if (lastOver != 0) {
                add = (lastOver - accToTest) / 2;
            } else if ((lastOver - allowedError) < accToTest && accToTest < (lastOver + allowedError)) {
                break;
            }//if none of these conditions are met, 10 is added to accToTest

            accToTest += add;
            turnIn--;
            lastTested = accToTest;

        }
        return accToTest;
    }

    /*TODO
     * slip vs acc
     */

    public static Twist2d getspeed() {
        long currentTime = System.nanoTime();
        double deltaTime = ((double) (currentTime - spreviousTime)) / 1.0e9; // Delta time in seconds

        if (deltaTime <= 0) {
            return new Twist2d(0, 0, 0); // Avoid division by zero
        }

        Pose2d currentPose = RobotContainer.odometry.getCurrentPos();

        // Calculate current velocity
        double vx = (currentPose.getX() - spreviousPose.getX()) / deltaTime;
        double vy = (currentPose.getY() - spreviousPose.getY()) / deltaTime;
        double vtheta = (currentPose.getRotation().getRadians() - spreviousPose.getRotation().getRadians()) / deltaTime;
        Twist2d currentVelocity = new Twist2d(vx, vy, vtheta);

        // Update state for the next call
        spreviousTime = currentTime;
        spreviousPose = currentPose;
        spreviousVelocity = currentVelocity;

        return currentVelocity;
    }

    public static Twist2d getAccel() {
        long currentTime = System.nanoTime();
        double deltaTime = ((double) (currentTime - previousTime)) / 1.0e9; // Delta time in seconds

        if (deltaTime <= 0) {
            return new Twist2d(0, 0, 0); // Avoid division by zero
        }

        Pose2d currentPose = RobotContainer.odometry.getCurrentPos();

        // Calculate current velocity
        double vx = (currentPose.getX() - previousPose.getX()) / deltaTime;
        double vy = (currentPose.getY() - previousPose.getY()) / deltaTime;
        double vtheta = (currentPose.getRotation().getRadians() - previousPose.getRotation().getRadians()) / deltaTime;
        Twist2d currentVelocity = new Twist2d(vx, vy, vtheta);

        // Calculate acceleration
        double ax = (currentVelocity.dx - previousVelocity.dx) / deltaTime;
        double ay = (currentVelocity.dy - previousVelocity.dy) / deltaTime;
        double atheta = (currentVelocity.dtheta - previousVelocity.dtheta) / deltaTime;
        Twist2d acceleration = new Twist2d(ax, ay, atheta);

        // Update state for the next call
        previousTime = currentTime;
        previousPose = currentPose;
        previousVelocity = currentVelocity;

        return acceleration;
    }

    /**
     * Tracks how much slip has accumulated since last reset
     *
     * @param resetSlipCalc resets the slip calculation if set to true
     * @return the accumulated slip
     */
    public static Pose2d getAccumulatedSlip(boolean resetSlipCalc) {
        Pose2d driveWheelPos; //the position from the drive wheel encoders
        Pose2d odometryPos;
        Pose2d slip;
        double slipOverTime, r, w, v;

        driveWheelPos = RobotContainer.driveWheelOdometry.GetPosition();
        odometryPos = RobotContainer.odometry.getCurrentPos();
        slip = driveWheelPos.relativeTo(odometryPos);

        if (resetSlipCalc) {
            RobotContainer.driveWheelOdometry.SetPosition(RobotContainer.odometry.getCurrentPos());
            RobotContainer.slipTimer.reset();
        }

        return slip;
    }

    ;

    /**
     * Calculates the rate of change of the slip
     *
     * @return A Twist2d representing the rate of change of the slip dx and dy are in units per
     * second, and dtheta is in radians per second.
     */
    public static Twist2d getSlip() {
        Pose2d currentSlip = getAccumulatedSlip(false);
        double deltaTime = RobotContainer.slipTimer.time() / 1000.0; // convert to seconds

        // If it's the first run or no time has passed, return a zero twist.
        if (isFirstDerivativeCalculation || deltaTime <= 0) {
            previousSlip = currentSlip;
            //RobotContainer.slipTimer.reset();
            isFirstDerivativeCalculation = false;
            return new Twist2d(0.0, 0.0, 0.0);
        }

        // Calculate the difference in slip
        double dx = (currentSlip.getX() - previousSlip.getX()) / deltaTime;
        double dy = (currentSlip.getY() - previousSlip.getY()) / deltaTime;
        double dtheta = (currentSlip.getRotation().getRadians() - previousSlip.getRotation().getRadians()) / deltaTime;

        // Update state for the next calculation
        previousSlip = currentSlip;
        RobotContainer.slipTimer.reset();


        return new Twist2d(dx, dy, dtheta);
    }

    public void updateData() {
        Odometry.Acceleration ActualAcceleration = RobotContainer.odometry.GetChassisAcceleration();
        ax = ActualAcceleration.ax;
        ay = ActualAcceleration.ay;

        Odometry.Speed ActualSpeed = RobotContainer.odometry.GetChassisSpeed();
        DriveWheelOdometry.Speed MeasuredSpeed = RobotContainer.driveWheelOdometry.GetChassisSpeed();
        slipX = ActualSpeed.vx - MeasuredSpeed.vx;
        slipY = ActualSpeed.vy - MeasuredSpeed.vy;

        writeDataIn -= 1;
        if (writeDataIn == 0) {
            writeDataIn = 25;
            linesWritten += 1;
            Twist2d slip = TestSlip.getSlip();
            //WriteToFile.appendToFile( String.format("%d%s%f%s%f%s%f%s%f", linesWritten, 't', RobotContainer.totalSlip, 'h', RobotContainer.highestSlip, 'x', RobotContainer.xSlip, 'y', RobotContainer.ySlip), FILE_NAME);
            System.out.println(String.format("%s%d%s%f%s%f%s%f%s%f%s", "slipData|", linesWritten, "|Ax,Sx,Ay,Sy|", ax, "|", slipX, "|", ay, "|", slipY));

        }

        /**
         * Writes the slip to a text file every 25 times the function is called
         */
//    @SuppressLint("DefaultLocale")
//    public static void writeSlip(){
//        writeDataIn -=1;
//        if (writeDataIn == 0){
//            writeDataIn = 25;
//            linesWritten +=1;
//            Twist2d slip = TestSlip.getSlip();
//            //WriteToFile.appendToFile( String.format("%d%s%f%s%f%s%f%s%f", linesWritten, 't', RobotContainer.totalSlip, 'h', RobotContainer.highestSlip, 'x', RobotContainer.xSlip, 'y', RobotContainer.ySlip), FILE_NAME);
//            System.out.println(String.format("%s%d%s%f%s%f%s%f%s%f%s%f%s%f%s%f", "slipData", linesWritten, 'a', RobotContainer.acc, 't', RobotContainer.totalSlip, 'h', RobotContainer.highestSlip, 'x', RobotContainer.xSlip, 'y', RobotContainer.ySlip, 'v',RobotContainer.xyacc.dx, 'w', RobotContainer.xyacc.dy));
//        }//+ 'x' + slip.dx +'y' + slip.dy
    }
}

