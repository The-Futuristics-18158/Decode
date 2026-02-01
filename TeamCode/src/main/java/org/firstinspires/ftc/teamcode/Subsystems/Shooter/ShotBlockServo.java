package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author superzokabear
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

    /**
     * block flywheel when intake on
     * <p>
     * Never change under penalty of a creative death, the servo value to 0.25 or greater
     */
    public void Block(){shotBlockServo.setPosition(0.17);}

    /**Unblock the flywheel
     * <p>
     * Never change under penalty of a creative death, the servo value to 0.25 or greater
     */
    public void Unblock(){shotBlockServo.setPosition(0.0);}

}