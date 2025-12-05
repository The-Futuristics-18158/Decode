package org.firstinspires.ftc.teamcode.Subsystems.Climb;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * Linear Slide Subsystem
 */
public class ClimbSubsystem extends SubsystemBase {

    // Initialize both motors
    private final DcMotorEx climbL;
    private final DcMotorEx climbR;


    /**
     * Place code here to initialize subsystem
     */
    public ClimbSubsystem() {

        // Creates the motors using the hardware map
        climbL = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "climb left");
        climbR = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "climb right");

        // Resets the encoders for both motors
        climbL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Turn the left motor in reverse to move the slide upwards
        climbL.setDirection(DcMotorSimple.Direction.REVERSE);
        climbR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Sets the motors PIDF values
        climbL.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        climbR.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);


        // Setting target to zero upon initialization
        climbL.setTargetPosition(0);
        climbR.setTargetPosition(0);

        // Puts the motors into position control mode
        climbL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

        //RobotContainer.DBTelemetry.addData("ClimbPose ", climb.getCurrentPosition());
        //RobotContainer.DBTelemetry.update();

    }

    // Using the var ticks sets the motor encoder ticks to a set position
    public void moveClimb(int ticks) {

        // Sets both motors to the ticks target position
        climbL.setTargetPosition(ticks);
        climbR.setTargetPosition(ticks);

        // Sets the power VERY IMPORTANT
        climbL.setPower(1);
        climbR.setPower(1);

    }

    public void moveTo(ClimbTargetHeight target) {moveClimb(target.getValue());}

}