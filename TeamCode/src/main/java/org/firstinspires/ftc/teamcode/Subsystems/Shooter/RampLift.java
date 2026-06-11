package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem to raise/lower intake ramp
 * raised position = 1.0
 * lowered position = 0.44
 *
 * @author superzokabear
 */
public class RampLift extends SubsystemBase {

    // Local objects and variables here
    private Servo RampServo;

    // servo positions
    private final double RaisedPosition = 1.0;
    private final double LoweredPosition = 0.44;

    /** Place code here to initialize subsystem */
    public RampLift() {

        // create servo and configure
        RampServo = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rampServo");

        // set default ramp position to low
        Lower();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
    }

    // place special subsystem methods here

    /**
     * Raises the intake ramp to shoot
     * <p>
     */
    public void Raise(){RampServo.setPosition(RaisedPosition);}

    /**Lowers the intake ramp
     * <p>
     */
    public void Lower(){RampServo.setPosition(LoweredPosition);}

    public boolean RampIsUp(){
        if (RampServo.getPosition() >= 1.0){
            return true;
        }else {
            return false;
        }
    }

}