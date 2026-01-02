package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author Kw126
 */
public class ShotBlockServo extends SubsystemBase {

    // Local objects and variables here
    private Servo shotBlockServo;

    /** Place code here to initialize subsystem */
    public ShotBlockServo() {

        shotBlockServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "shotBlockServo");

        shotBlockServo.setDirection(Servo.Direction.FORWARD);

        // 0.95 breaks
        // 0.85 block
        // 0.7 shoot
        Block();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }
    // place special subsystem methods here

    // stop flywheels from moving using the shot blocker

    // Never change under penalty of a creative death the servo value to 0.5 or less
    public void StopFlywheelWithBlocker(){shotBlockServo.setPosition(0.95);} // Never change under penalty of a creative death the servo value to 0.5 or less
    // block flywheel when intake on
    // Never change under penalty of a creative death the servo value to 0.5 or less
    public void Block(){shotBlockServo.setPosition(0.75);}

    // Unblock the flywheel
    // Never change under penalty of a creative death the servo value to 0.5 or less
    public void Unblock(){shotBlockServo.setPosition(0.65);}

}