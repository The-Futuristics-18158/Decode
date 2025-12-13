package org.firstinspires.ftc.teamcode.Subsystems.Climb;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
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

    // Controller PIDF gains
    private double PGain = 0.007;    // was 0.005
    private double FsGain = 0.50;    // was 0.5

    // limit for both motors
    private double MaxPower = 0.70;

    // PID controllers for each motor
    private double LTarget;
    private double RTarget;

    // motor OFF boolean
    private boolean MotorOff;

    /**
     * Place code here to initialize subsystem
     */
    public ClimbSubsystem() {

        // Creates the motors using the hardware map
        climbL = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        climbR = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightLiftMotor");

        // Turn the left motor in reverse to move the slide upwards
        climbL.setDirection(DcMotorSimple.Direction.REVERSE);
        climbR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Resets the encoders for both motors
        climbL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // motor in open-loop mode (power mode)
        climbL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // apply zero power to motors
        climbL.setPower(0.0);
        climbR.setPower(0.0);

        // set targets to default
        LTarget = 0.0;
        RTarget = 0.0;

        // motor off by default
        MotorOff = true;
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

        // Left Controller
        double LError = LTarget - climbL.getCurrentPosition();
        double LPower = FsGain + PGain*LError;
        LPower = Math.max(0.0, LPower);
        LPower = Math.min(MaxPower, LPower);
        if (LError < 0.0)
            LPower = 0.0;

        // Right Controller
        double RError = RTarget - climbR.getCurrentPosition();
        double RPower = FsGain + PGain*RError;
        RPower = Math.max(0.0, RPower);
        RPower = Math.min(MaxPower, RPower);
        if (RError < 0.0)
            RPower = 0.0;

        // set motor powers
        if (MotorOff)
            { LPower = 0.0; RPower = 0.0; }
        //RobotContainer.RCTelemetry.addData("lpower", LPower);
        //RobotContainer.RCTelemetry.addData("rpower", RPower);
        climbL.setPower(LPower);
        climbR.setPower(RPower);
    }

    // only move as fast as slowest motor
    public void moveClimb() {

        // Sets both motors to the target position
        int lowestTics = Math.min(climbL.getCurrentPosition(), climbR.getCurrentPosition());
        int targetPos = lowestTics + 50;
        if (targetPos > 2090){
            targetPos = 2090;// 2090
        }
        LTarget = targetPos;
        RTarget = targetPos;
        MotorOff = false;
    }

    public void climbStop(){
        // set targets to current position to stop motors
        LTarget = climbL.getCurrentPosition();
        RTarget = climbR.getCurrentPosition();
        MotorOff = true;
    }



}