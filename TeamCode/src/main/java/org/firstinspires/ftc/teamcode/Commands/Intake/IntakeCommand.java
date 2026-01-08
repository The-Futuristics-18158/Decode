package org.firstinspires.ftc.teamcode.Commands.Intake;

import static kotlin.concurrent.TimersKt.timer;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class IntakeCommand extends CommandBase {

    private ElapsedTime timer;

    boolean finished;

    // constructor
    public IntakeCommand() {

        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.intake);
        timer = new ElapsedTime();
        timer.reset();


    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        timer.reset();

        finished = false;
        RobotContainer.uptake.LowerLeftUptake();
        RobotContainer.uptake.LowerRightUptake();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        if (timer.seconds()>5.0 || (RobotContainer.artifactCamera.IsLeftPresent() && RobotContainer.artifactCamera.IsRightPresent() && RobotContainer.artifactCamera.IsBottomPresent())){

            //wait.reset();
//            if(wait.seconds()<=5.0){
//                RobotContainer.intake.intakeBackup();
//
//            }
            RobotContainer.intake.intakeStop();

            finished = true;
        }else {
            RobotContainer.intake.intakeRun();
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

        RobotContainer.intake.intakeStop();

    }

}