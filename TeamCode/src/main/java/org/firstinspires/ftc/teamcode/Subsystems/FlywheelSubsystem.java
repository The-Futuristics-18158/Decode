package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author Zoe
 */
public class FlywheelSubsystem extends SubsystemBase {

    // Local objects and variables here
    private final DcMotorEx flywheelMotor;

    /** Place code here to initialize subsystem */
    public FlywheelSubsystem() {
        // Creates the motor using the hardware map
        flywheelMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "flywheel");
        // Resets the encoders
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Sets motor direction
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Sets the motors PIDF values
        flywheelMotor.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        // Sets velocity
        flywheelMotor.setVelocity(0.0);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // Place special subsystem methods here

    // A method to convert from RPM to motor velocity in ticks per second
    public double RPMToVelocity (double RPM){
        // 60 seconds in one minute
        // 28 ticks per revolution of the motor at the motor
        // 12:1 gear reduction so 1 rotation at the shaft is 12 rotations at the motor
        return RPM / 60.0 * 28 * 12.0;
    }

    public void flywheelSpeed(double RPM){
        // Setting velocity using the RPMToVelocity methode
        flywheelMotor.setVelocity(RPMToVelocity(RPM));
    }

}