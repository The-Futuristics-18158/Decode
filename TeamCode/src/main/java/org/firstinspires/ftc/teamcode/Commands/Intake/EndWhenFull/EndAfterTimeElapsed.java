package org.firstinspires.ftc.teamcode.Commands.Intake.EndWhenFull;


import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * This command ends only when the robot contains two artifacts in it
 *
 * @author knutt5
 */
public class EndAfterTimeElapsed extends CommandBase {
    private ElapsedTime timer;
    boolean finished;
    // constructor
    public EndAfterTimeElapsed() {

        // Note: this command does not require any subsystems
        //addRequirements();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        timer.reset();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        if (timer.seconds()>2.5){ //May need to adjust timeout
            finished = true;
            //RobotContainer.intake.intakeStop();
        }
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return finished;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}