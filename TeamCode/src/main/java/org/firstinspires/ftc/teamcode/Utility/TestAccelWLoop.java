package org.firstinspires.ftc.teamcode.Utility;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;

import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * class that contains an automatic function that determines max acceleration that utilizes a while loop
 *
 * @author kw126
 * */
public class TestAccelWLoop {
     /*TODO
     * add code to change movement direction when turnIn = 0  line 46
     *
     */

    /**
     * Function that can be used to determine the fasted acceleration the robot can use without creating slip
     *
     * @param maxSpeed the max speed the robot is allowed to reach during the test movement
     * @param accToTest the starting acceleration to test
     * @param distance the distance the robot will travel during each test movement
     * @param maxBeforeTurn the max number of test movements before the robot direction flips
     * @param allowedError an acceptable amount of slip
     *
     * @return the fasted acceleration the robot can use without creating slip
     */
    public int testAcc(int maxSpeed, int accToTest, int distance, int maxBeforeTurn, int allowedError){

        //declare variables
        int add = 10; // how much to add to the test acceleration. if the acceleration is too fast the amount to add will be changed.
        boolean tooFast = false; //was last tested too fast
        int turnIn = maxBeforeTurn-1;
        boolean goForward = true;
        int lastTested = 0;//the last acceleration that was tested
        int lastOver = 0;//the last acceleration that was too fast
        Pose2d driveWheelPosB4; //the position from drive wheel encoders before the movement test
        Pose2d driveWheelPosAft; //the position from the drive wheel encoders after the movement test

        //WHILE ()
        while(true) {
            //code to make sure robot does not drive off mat
            if (turnIn == 0){
                if (goForward){
                    goForward = false;
                    //change direction
                    }
                else{
                    goForward = true;
                    //change direction
                }
                turnIn = maxBeforeTurn;
            }

            driveWheelPosB4 = RobotContainer.driveWheelOdometry.GetPosition();

            // create command
            MoveToPose accelMove = new MoveToPose(maxSpeed, accToTest, RobotContainer.odometry.getCurrentPos().plus(new Transform2d()));//add to scheduler

            // schedule command (add to scheduler)
            accelMove.schedule();

            // wait for path command to finish
            while (!accelMove.isFinished()) {
            }

            // do some stuff to compare drive wheel odometry with pod odometry
            //compare odometry to determine if the acceleration is too fast
            driveWheelPosAft = RobotContainer.driveWheelOdometry.GetPosition();

            // figure out some new parameters
            if(tooFast){
                add = (accToTest - lastTested) / 2;
                lastOver = accToTest;
            } else if (lastOver != 0) {
                add = (lastOver - accToTest)/2;
            } else if ((lastOver-allowedError) < accToTest && accToTest < (lastOver + allowedError)){
                break;
            }//if none of these conditions are met, 10 is added to accToTest

            accToTest += add;
            turnIn--;
            lastTested = accToTest;

        }
        return accToTest;
    }

}
