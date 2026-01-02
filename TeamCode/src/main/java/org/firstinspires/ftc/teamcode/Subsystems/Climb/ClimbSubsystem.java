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
        climbL = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        climbR = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightLiftMotor");

        // Resets the encoders for both motors
        climbL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Turn the left motor in reverse to move the slide upwards
        climbL.setDirection(DcMotorSimple.Direction.REVERSE);
        climbR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Sets the motors PIDF values
        climbL.setVelocityPIDFCoefficients(20.0, 0.2, 0.001, 10.0);
        climbR.setVelocityPIDFCoefficients(20.0, 0.2, 0.001, 10.0);


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

    // only move as fast as slowest motor
    public void moveClimb() {

        // Sets both motors to the target position
        double rollAngle = RobotContainer.gyro.getRollAngle();
        double leftTics = climbL.getCurrentPosition();
        double rightTics = climbR.getCurrentPosition();
        double motorDifference = Math.abs(leftTics - rightTics);
        // ~1500 tics to top


        if (leftTics > rightTics){
            //reduce power of left motor
            climbL.setPower(Math.max(1.0 - (motorDifference * 0.01), 0.0));
        }else if (rightTics > leftTics) {
            //reduce power of right motor
            climbR.setPower(Math.max(1.0 - (motorDifference * 0.01), 0.0));
        }else{
            climbL.setPower(1.0);
            climbR.setPower(1.0);
        }
    }

    public void climbStop(){
        climbL.setPower(0);
        climbR.setPower(0);

    }



}