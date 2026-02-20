package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
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

    private final double TICKS_PER_ROTATION = 28.0;
    private final double INV_TICKS_PER_ROTATION = 1.0 / TICKS_PER_ROTATION;
    // f and p gain units in mtr power/(ticks/s)
    private double fgain = 0.000357 * TICKS_PER_ROTATION;     // no load ideal = 0.000357
    private double pgain = 0.00025 * TICKS_PER_ROTATION;
    private double igain = 0.0;

    private double ierror;
    private double TargetSpeed;


    /** Place code here to initialize subsystem */
    public IntakeSubsystem() {
        // Creates the motor using the hardware map
        intakeMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        // Sets motor direction
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Set motor power
        intakeMotor.setPower(0.0);
        // by default, set target speed to 0
        TargetSpeed = 0.0;
        // reset PIF controller
        ierror=0.0;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // sets motor speed in rps (=28xticks/s)
        double CurrentSpeed = intakeMotor.getVelocity() * INV_TICKS_PER_ROTATION;
        double error = TargetSpeed - CurrentSpeed;

        // integrated error
        ierror += igain * error;

        // special case - if target is -ve, then we are jogging back - use higher P
        double PExtra = 1.0;
        if (TargetSpeed < 0.0)
            PExtra = 1.5;

        // set intake motor power  (PIF controller)
        intakeMotor.setPower(PExtra * pgain * error + fgain * TargetSpeed + ierror);

        RobotContainer.Panels.FTCTelemetry.addData("ShootSpeed", CurrentSpeed);
        RobotContainer.Panels.FTCTelemetry.addData("Target", TargetSpeed);
        RobotContainer.Panels.FTCTelemetry.update();
    }

    // Place special subsystem methods here

    /** Sets speed of intake in motor rps */
    public void intakeSetSpeed(double speed) { TargetSpeed=speed; }

    /**Run the intake at set speed (rps)*/
    public void intakeRun(){ TargetSpeed=100.0;}

    /**Run the intake at set speed (rps)*/
    public void intakeRunReducedSpeed() { TargetSpeed=40.0; }

    /**Run the intake at set speed (rps)*/
    public void intakeReverse(){ TargetSpeed = -15.0; }

    /**Stop intake*/
    public void intakeStop(){ TargetSpeed=0.0;}

    /** returns motor position in encoder ticks */
    public double GetMotorPostion() { return intakeMotor.getCurrentPosition(); }

}