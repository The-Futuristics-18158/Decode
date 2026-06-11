package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to Operate Shot Blocker
 * Open position = 0.35
 * Close position = 0.0
 *
 * @author superzokabear
 */
public class ShotBlock extends SubsystemBase {

    // Local objects and variables here
    private Servo shotBlockServo;

    /** Place code here to initialize subsystem */
    public ShotBlock() {

        // create servo and configure
        shotBlockServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "shotBlockServo");
        shotBlockServo.setDirection(Servo.Direction.FORWARD);

        // set default blocker position
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
    public void Block(){shotBlockServo.setPosition(0.0);}

    /**Unblock the flywheel
     * <p>
     * Never change under penalty of a creative death, the servo value to 0.25 or greater
     */
    public void Unblock(){shotBlockServo.setPosition(0.35);}

    public boolean ShotBlocked(){
        if (shotBlockServo.getPosition() >= 0.0 && shotBlockServo.getPosition() < 0.35){
            return true;
        }else {
            return false;
        }
    }

}