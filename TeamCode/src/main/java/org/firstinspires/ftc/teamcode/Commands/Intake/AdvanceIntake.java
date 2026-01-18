package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class AdvanceIntake extends CommandBase {

    private ElapsedTime timer;
    boolean finished;

    // constructor
    public AdvanceIntake() {

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
        if (timer.seconds()>2.5 || (RobotContainer.artifactCamera.IsLeftPresent() &&
                RobotContainer.artifactCamera.IsRightPresent())){
            finished = true;
            RobotContainer.intake.intakeStop();
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