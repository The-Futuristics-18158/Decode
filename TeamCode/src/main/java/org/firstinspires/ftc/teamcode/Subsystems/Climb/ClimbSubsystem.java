package org.firstinspires.ftc.teamcode.Subsystems.Climb;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * Linear Slide Subsystem
 */
public class ClimbSubsystem extends SubsystemBase {

    // Initialize both motors
    private final DcMotorEx climbL;
    private final DcMotorEx climbR;
    private  final Rev2mDistanceSensor climbSensor;

    /**
     * Place code here to initialize subsystem
     */
    public ClimbSubsystem() {

        climbSensor = RobotContainer.ActiveOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "climbDistance");
        climbSensor.initialize();
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
        climbL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        double climbDistance = climbSensor.getDistance(DistanceUnit.INCH);
        RobotContainer.RCTelemetry.addData("Climb distance", climbDistance);
        double rollAngle = RobotContainer.gyro.getRollAngle();
        RobotContainer.RCTelemetry.addData("roll", rollAngle);
        //RobotContainer.DBTelemetry.addData("ClimbPose ", climb.getCurrentPosition());
        //RobotContainer.DBTelemetry.update();

    }

    // only move as fast as slowest motor
    public void moveClimb() {

        // Sets both motors to the target position
        double rollAngle = RobotContainer.gyro.getRollAngle();
        double climbDistance = climbSensor.getDistance(DistanceUnit.INCH);
        double angleDifference = (Math.abs(rollAngle) - 0.5) / 1.5;
        // ~1500 tics to top
        // when right side is low, roll is positive. left side low = roll negative
        if(climbDistance < 20.0){

            if (rollAngle <= - 0.5){
                //reduce power of left motor
                climbR.setPower(Math.max(1.0 - (angleDifference), 0.0));
                climbL.setPower(1.0);
            }else if (rollAngle >= 0.5) {
                //reduce power of right motor
                climbL.setPower(Math.max(1.0 - (angleDifference), 0.0));
                climbR.setPower(1.0);
            }else{
                climbL.setPower(1.0);
                climbR.setPower(1.0);
            }
        }else{
            climbStop();
        }

    }

    public void climbStop(){
        climbL.setPower(0);
        climbR.setPower(0);

    }



}