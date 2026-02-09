package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author superzokabear
 */
// @Configurable
public class HoodTiltSubsystem extends SubsystemBase {

    // Local objects and variables here
    public static double hoodPosition;
    private Servo leftTilt;
    private Servo rightTilt;

    /** Place code here to initialize subsystem */
    public HoodTiltSubsystem() {

        leftTilt = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftHoodServo");
        rightTilt = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightHoodServo");

        leftTilt.setDirection(Servo.Direction.FORWARD);
        rightTilt.setDirection(Servo.Direction.FORWARD);

        hoodPosition = MinLeftAngle;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        //SetHoodPosition(hoodPosition);
    }

    // place special subsystem methods here
    // Raise left or right hood servo
    public double MaxLeftAngle = 0.57; // No higher under penalty of a creative death
    public double MaxRightAngle = 0.43; // No higher under penalty of a creative death
    public double MinLeftAngle = 0.0;
    public double MinRightAngle = 1.0;

//    public void MaxRaiseLeftTilt(){leftTilt.setPosition(MaxLeftAngle);}
//    public void MaxRaiseRightTIlt(){
//        rightTilt.setPosition(MaxRightAngle);
//    }
//
//    // Lower Left or right hood servo
//    public void LowerLeftTilt(){leftTilt.setPosition(MinLeftAngle);}// No negative values possible
//    public void LowerRightTilt(){
//        rightTilt.setPosition(MinRightAngle);
//    }


    /**Sets shooter hood position
     * @param position Desired position between 0.0 and 0.57
     */
    public void SetHoodPosition(double position){
        double pos = position;
        if (pos < MinLeftAngle){ pos = MinLeftAngle;}
        if (pos > MaxLeftAngle){ pos = MaxLeftAngle;}
        leftTilt.setPosition(pos);
        rightTilt.setPosition(1.0 - pos);
    }


}