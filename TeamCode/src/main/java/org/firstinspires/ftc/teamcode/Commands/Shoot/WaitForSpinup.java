package org.firstinspires.ftc.teamcode.Commands.Shoot;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * Command waits for shooter to spin up - up to a maximum time
 *
 * @author knutt5
 */
public class WaitForSpinup extends CommandBase {

    double maxtime;
    ElapsedTime time;

    // Constructor
    public WaitForSpinup() {
        time = new ElapsedTime();
    }


    // This method is called once when command is started
    @Override
    public void initialize() {
        // start timer;
        time.reset();

        // determine maximum time before command timeout
        // assume shooter takes ~4s/3,000rpm. Note: time to spin up varies with square of speed
        double x = Math.max(RobotContainer.shooter.GetFlyWheelTargetSpeed() / 3000.0, 0.0);
        maxtime = x * x * 4.0;
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        // command is finished when shooter reaches 98% of speed or max time is exceeded
        return (RobotContainer.shooter.GetFlyWheelSpeed() >=0.98*RobotContainer.shooter.GetFlyWheelTargetSpeed() ||
                time.seconds() >= maxtime);
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}