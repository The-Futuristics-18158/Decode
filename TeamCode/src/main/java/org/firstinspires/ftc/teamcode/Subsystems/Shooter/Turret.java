package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Turret Subsystem
 * @author superzokabear
 * @author Kw126
 */
public class Turret extends SubsystemBase {

    // Initialize motor
    private final DcMotorEx turret;

    // ticks to degrees scaling factor  - ken gets 266/180 = 1.4777
    // jeff previously measured 462/360. Testing appears to show 266/180 is closed to value needed
    private final double DEGREES_TO_TICKS = 266.0/180.0;
    private final double TICKS_TO_DEGREES = 1.0 / DEGREES_TO_TICKS;

    // min and max turret angle limits
    private final double MIN_ANGLE_DEG = -135.0;
    private final double MAX_ANGLE_DEG = 135.0;

    // turret current target (in degrees from center position)
    private double TurretTargetDegrees;

    // encoder offset used to 'zero' the turret
    private static int EncoderOffset = 0;

    // turret manual reset mode
    private boolean ManualResetMode;


    /**
     * Place code here to initialize subsystem
     */
    public Turret() {
        // Creates the motor using the hardware map
        turret = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "turretMotor");

        // Resets the encoder for motor
        // turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Motor direction
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setting target to zero upon initialization
        turret.setTargetPosition(0);
        TurretTargetDegrees = 0.0;

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setPower(1.0);

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

        // bound the angle
        if (TurretTargetDegrees<MIN_ANGLE_DEG)  TurretTargetDegrees=MIN_ANGLE_DEG;
        if (TurretTargetDegrees>MAX_ANGLE_DEG)  TurretTargetDegrees=MAX_ANGLE_DEG;


        RobotContainer.telemetrySubsystem.addData("encoder",turret.getCurrentPosition(), true);
        RobotContainer.telemetrySubsystem.addData("offset",EncoderOffset, true);
        RobotContainer.telemetrySubsystem.addData("position",getTurretTicks(), true);

        // if robot is not in auto or teleop, then force turret to be unpowered
        if ((RobotContainer.GetCurrentMode() != RobotContainer.Modes.Auto &&
                RobotContainer.GetCurrentMode() != RobotContainer.Modes.TeleOp) ||
                ManualResetMode)
        {
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setPower(0.0);
        }
        else
        {
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(1.0);


            // determine target position in ticks
            int targetPosition = (int) (TurretTargetDegrees * DEGREES_TO_TICKS);

            // determine current position error (in degrees)
            double turretRemainingError = TurretTargetDegrees - getTurretDegrees();


            // create variable PI control for remaining error and smooth operation
            double variableP = -0.47 * turretRemainingError + 99.37; // -0.71 * x + 107.86 was a bit too aggressive
            double variableI = -0.047 * turretRemainingError + 9.937; // -0.071 * x + 10.786 was a bit too aggressive

            // enable mins
            variableP = Math.max(15.0, variableP);
            variableI = Math.max(1.5, variableI);

            // enable maxes
            variableP = Math.min(90.0, variableP);
            variableI = Math.min(15.0, variableI);
            //variableI = Math.min(0.0, variableI);

            // set variable PI
            turret.setVelocityPIDFCoefficients(variableP, variableI, 0.0, 0.0);

            // move the turret motor
            turret.setTargetPosition(targetPosition+EncoderOffset);

            // default positional control PID values
            // Jun 6/2026 KN
            // p=10.0;
            // i=0.05
            // d=0.0

            turret.setPositionPIDFCoefficients(14.0);

            // temporary
            //double p = turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p;
            //double i = turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i;
            //double d = turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d;
            //RobotContainer.telemetrySubsystem.addData("default P",p, true);
            //RobotContainer.telemetrySubsystem.addData("default I",i, true);
            //RobotContainer.telemetrySubsystem.addData("default D",d, true);

            // temporary - display turret position and target on panels for graphing
            PanelsTelemetry.INSTANCE.getTelemetry().addData("TurretTargetDeg", getTurretTargetDegrees());
            PanelsTelemetry.INSTANCE.getTelemetry().addData("TurretDeg", getTurretDegrees());
        }
    }

    // reset turret position to zero when facing forward
    public void ResetTurretPositionStraight()
    {
        // set new encoder offset to new overall position is zeroed
        EncoderOffset = -turret.getCurrentPosition();
    }

    // puts turret in manual operating mode for purposes of resetting position
    public void SetManualMode(boolean enable)
    {
        ManualResetMode = enable;
    }

    // used to manuall cotrol turret
    public void SetManualSpeed(double pwr)
    {
        if (ManualResetMode)
        {
            if (pwr>0.5) pwr=0.5;
            if (pwr<-0.5) pwr=-0.5;
            turret.setPower(pwr);
        }
    }


    /** Switches off Turret */
    public void turretStop(){
        turret.setPower(0.0);
    }

    /**
     * Moves turret to specified angle
     * Angle bounded by MIN_ANGLE_DEG and MAX_ANGLE_DEG
     * 0deg is pointed forwarded (in robot space)
     * @param TargetDegrees target angles in degrees
     */
    public void moveTurret(double TargetDegrees) {
        TurretTargetDegrees = TargetDegrees;
    }

    /** returns current turret target position
     * @return turret position in degrees
     */
    public double getTurretTargetDegrees(){
        return getTurretTargetTicks() * TICKS_TO_DEGREES;
    }

    /** returns current turret position
     * @return turret position in degrees
     */
    public double getTurretDegrees(){
        return getTurretTicks() * TICKS_TO_DEGREES;
    }

    /** returns current turret target position
     * @return turret position in encoder ticks
     */
    public int getTurretTargetTicks(){
        return turret.getTargetPosition()+EncoderOffset;
    }

    /** returns current turret position
     * @return turret position in encoder ticks
     */
    public double getTurretTicks(){
        return turret.getCurrentPosition()+EncoderOffset;
    }


}
