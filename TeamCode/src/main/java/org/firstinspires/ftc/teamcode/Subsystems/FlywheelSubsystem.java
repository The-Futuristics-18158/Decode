package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.GoalTargeting;

/**
 * Place description of subsystem here
 *
 * @author Zoe
 */
@Configurable
public class FlywheelSubsystem extends SubsystemBase {

    // Local objects and variables here
    private final DcMotorEx flywheelMotor;

    // constants
    private final double MAXRPM = 6000.0;
    private final double TICKSPStoRPM = (1/28.0)*60.0;

    // target speed
    public static double TargetSpeed;  // When disabling dashboard/panels turn back to privet.
    public static double CurrentSpeed; // When disabling dashboard/panels turn back to privet.
    // PIF Controller Gains
    private final double FsGain = 0.0;
    private final double FvGain = 0.00021; // was 0.0002 // unit=power/rpm   initial value=1.0/6000rpm=0.00016667
    public final double PGain = 0.0012;// was 0.0003
    public final double IGain = 0.0002;

    // integrated error
    private double IError;
    private ElapsedTime timer;


    /** Place code here to initialize subsystem */
    public FlywheelSubsystem() {
        // create motor
        flywheelMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // important! - set motor to coast mode - only works for 0 power
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // motor is initially off
        flywheelMotor.setPower(0.0);

        // reset integrated error
        timer = new ElapsedTime();
        timer.reset();
        IError=0.0;

        // reset target speed (rpm)
        TargetSpeed=0.0;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // our current speed
        CurrentSpeed = flywheelMotor.getVelocity() * TICKSPStoRPM;

        // our current speed error
        double SpeedError = TargetSpeed - CurrentSpeed;

        // integrated error
        // determine time since last iteration
        double dt = timer.seconds();
        timer.reset();
        // integrate speed error
        IError += IGain * SpeedError * 0.02;
        // anti-windup to prevent overshoots
        if (SpeedError < -50.0 && IError > 0.0)
            IError *=0.90;
        if (SpeedError > 50.0 && IError < 0.0)
            IError *=0.90;
        // integrated error limiter
        if (IError > 0.15) IError=0.15;
        if (IError < -0.1) IError=-0.1;

        // PIF controller
        double NewPower = FsGain +                    // static feedforward
                FvGain * TargetSpeed +      // speed feedforward
                PGain * SpeedError +        // proportional gain
                IError;                     // integrated error
        // only drive motor in positive direction, otherwise let it coast
        if (SpeedError>=-50.0)
            flywheelMotor.setPower(NewPower);
        else
            flywheelMotor.setPower(0.0);

        RobotContainer.Panels.FTCTelemetry.addData("Speed", CurrentSpeed);
        RobotContainer.Panels.FTCTelemetry.addData("Target", TargetSpeed);
        RobotContainer.Panels.FTCTelemetry.update();
    }

    // Place special subsystem methods here


    public void SetFlywheelSpeed(double RPM){
        // Setting velocity using the RPMToVelocity methode
        //TargetSpeed = RobotContainer.targeting.CalculateSpeed();
        TargetSpeed = RPM;

    }

    // returns current flywheel speed in rpm
    public double GetFlyWheelSpeed() {
        return CurrentSpeed;
    }

    // returns target flywheel speed in rpm
    public double GetFlyWheelTargetSpeed() {
        return TargetSpeed;
    }

}