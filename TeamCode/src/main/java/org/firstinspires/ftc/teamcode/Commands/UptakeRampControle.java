package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class UptakeRampControle extends CommandBase {

    private ElapsedTime timer;
    private double seconds;
    private boolean timeout;

    // constructor
    public UptakeRampControle(){this (0.0, false);}
    public UptakeRampControle(double time){this (time, true);}
    public UptakeRampControle(double time, boolean timeoutEnabled) {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.uptakeSensor);
        addRequirements(RobotContainer.ramplift);

        timeout = timeoutEnabled;

        timer = new ElapsedTime();
        seconds = time;
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
         RobotContainer.ramplift.Raise();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return RobotContainer.uptakeSensor.isUpakeArtifactPresent() ||
                (timeout && timer.seconds() > seconds);
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.ramplift.Lower();
    }

}