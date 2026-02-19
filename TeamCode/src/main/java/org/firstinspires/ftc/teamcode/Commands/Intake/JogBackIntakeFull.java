package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class JogBackIntakeFull extends CommandBase {


    double TargetPosition;

    // unit for P-gain:  motor rps / motor ticks
    // note: 100 is gain
    // 1/28 * 1/5.0 is the gear ratio as we specify target turns in rotations of intake shaft
    final double Pgain = 100.0 * (1/28.0)*(1/5.0);

    // number of intake shaft turns to jog back by.  i.e. jog back a fixed 0.15 turns
    final double JogBackTurns = 0.15;


    // constructor
    public JogBackIntakeFull() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.intake);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // current robot position (in encoder ticks)
        double currentPos = RobotContainer.intake.GetMotorPostion();

        if (RobotContainer.artifactCamera.IsOverloadPresent())
            // we are overloaded - set target position at 0.4 rotations back
            // note: 28.0*5.0 factor converts to ticks
            TargetPosition = currentPos - JogBackTurns*28.0*5.0;
        else
            // set target to current pos (=zero error) - will turn off intake
            TargetPosition = currentPos;

        // position error (in motor ticks)
        double error = TargetPosition - RobotContainer.intake.GetMotorPostion();

        // Set speed of intake (in motor rps)
        RobotContainer.intake.intakeSetSpeed(Pgain * error);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.intake.intakeStop();
    }

}