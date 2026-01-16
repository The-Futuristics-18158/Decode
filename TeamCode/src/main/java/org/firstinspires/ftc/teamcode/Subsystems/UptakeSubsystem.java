package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author superzokabear
 */
public class UptakeSubsystem extends SubsystemBase {

    // Local objects and variables here
    private Servo leftUptake;
    private Servo rightUptake;

    /** Place code here to initialize subsystem */
    public UptakeSubsystem() {

        leftUptake = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "leftUptakeServo");
        rightUptake = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "rightUptakeServo");

        leftUptake.setDirection(Servo.Direction.FORWARD);
        rightUptake.setDirection(Servo.Direction.FORWARD);

        LowerLeftUptake();
        LowerRightUptake();

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here
    // Raise left or right uptake
    public void RaiseLeftUptake(){
        leftUptake.setPosition(0.03);
    }
    public void RaiseRightUptake(){
        rightUptake.setPosition(0.15);
    }
    // Lower Left or right uptake
    public void LowerLeftUptake(){
        leftUptake.setPosition(0.18);
    }
    public void LowerRightUptake(){
        rightUptake.setPosition(0.0);
    }
}