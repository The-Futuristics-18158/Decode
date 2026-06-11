package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.RobotContainer;


// Command to enable driver to move turret for purposes of resetting position

// command template
public class ManualTurretMoveForResetting extends CommandBase {

    // constructor
    public ManualTurretMoveForResetting() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
        addRequirements(RobotContainer.targeting);
        addRequirements(RobotContainer.turret);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        RobotContainer.targeting.EnableAutoTargeting(false);
        RobotContainer.turret.SetManualMode(true);
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        double dX = -RobotContainer.ActiveOpMode.gamepad1.left_stick_x;
        RobotContainer.turret.SetManualSpeed(dX*0.5);
        RobotContainer.turret.ResetTurretPositionStraight();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        RobotContainer.targeting.EnableAutoTargeting(true);
        RobotContainer.turret.SetManualMode(false);
    }

}