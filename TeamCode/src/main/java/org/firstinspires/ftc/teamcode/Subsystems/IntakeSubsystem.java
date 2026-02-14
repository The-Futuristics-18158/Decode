package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Setting up the intake subsystem.<p>
 * This makes the robot know that the intake motor exists.
 * This motor should run continuously not to a specific position.
 * Therefor this subsystem only has start and stop functions.
 * @author superzokabear
 */
public class IntakeSubsystem extends SubsystemBase {

    // Local objects and variables here
    private final DcMotorEx intakeMotor;



    /** Place code here to initialize subsystem */
    public IntakeSubsystem() {
        // Creates the motor using the hardware map
        intakeMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        // Resets the encoders
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Sets motor direction
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Setting power to zero upon initialization
        intakeMotor.setPower(0);


    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }
    // Place special subsystem methods here

    // Run the intake at set percentage of motor voltage
    public void intakeRun(){
        intakeMotor.setPower(1.0);
    }

    // used to back out a ball a little bit
    public void intakeReverse() {
        intakeMotor.setPower(-0.4);
    }

    // Stop intake
    public void intakeStop(){
        intakeMotor.setPower(0.0);
    }



//    public void intakeBackup(){
//        intakeMotor.setPower(-1.0);
//
//
//    }
}