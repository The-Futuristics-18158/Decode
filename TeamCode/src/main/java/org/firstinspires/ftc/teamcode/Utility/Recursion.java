package org.firstinspires.ftc.teamcode.Utility;

import com.arcrobotics.ftclib.command.old.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Transform2d;

import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;

/**
 * automatic function that determines max acceleration
 *
 * @author kw126
 * */
public class Recursion {//change name

    public int test(int maxSpeed, int testAcc, int maxBeforeTurn, boolean goForward, int distance, int lastOver){//change name

        //declare variables
        int add = 10; // how much to add to the test acceleration. if the acceleration is too fast the amount to add will be changed.
        boolean tooFast;
        int turnIn = maxBeforeTurn-1;
        int lastTested = 0;

        //check if the movement direction needs to be switched

        //get wheel position before moving

        //determine the direction the robot is facing and if you add to X or Y to move the robot forward

        //WHILE ()

        // create command
        MoveToPose katie = new MoveToPose(maxSpeed, testAcc, RobotContainer.odometry.getCurrentPos().plus(new Transform2d()));//add to scheduler

        // schedule command (add to scheduler)
        katie.schedule();

        // wait for path command to finish
        while (!katie.isFinished()) {}

        // do some stuff to compare drive wheel odometry with pod odometry
        // figure out some new parameters

        // END WHILE





        //get wheel positions after moving

        //compare odometry to determine if the acceleration is too fast

        /*if the acceleration is too fast
        * add = (testAcc - lastTested) / 2
        * return testAcc
        */
        return testAcc(maxSpeed, testAcc, testAcc+add, turnIn-1, maxBeforeTurn, goForward, distance, lastOver);

        /*if the acceleration is not too fast and lastover != 0
        * add = (lastOver - testAcc)/2
        */

        /*if acceleration is not too fast and within an acceptable range of lastOver
        * return int
        */

        /*else
        * return testAcc
        */
    }

    private int testAcc(int maxSpeed, int lastTested, int testAcc, int turnIn, int maxBeforeTurn, boolean goForward, int distance, int lastOver){

        return 0;
    };
}
