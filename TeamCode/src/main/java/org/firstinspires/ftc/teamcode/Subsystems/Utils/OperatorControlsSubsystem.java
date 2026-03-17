package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * methods to manage operator controls
 *
 * @author Kw126
 */
public class OperatorControlsSubsystem extends SubsystemBase {

    // Local objects and variables here

    /** Place code here to initialize subsystem */
    public OperatorControlsSubsystem() {

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        if (RobotContainer.artifactsInRamp == 0)
            RobotContainer.telemetrySubsystem.ascii_0(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 1)
            RobotContainer.telemetrySubsystem.ascii_1(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 2)
            RobotContainer.telemetrySubsystem.ascii_2(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 3)
            RobotContainer.telemetrySubsystem.ascii_3(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 4)
            RobotContainer.telemetrySubsystem.ascii_4(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 5)
            RobotContainer.telemetrySubsystem.ascii_5(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 6)
            RobotContainer.telemetrySubsystem.ascii_6(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 7)
            RobotContainer.telemetrySubsystem.ascii_7(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 8)
            RobotContainer.telemetrySubsystem.ascii_8(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
        else if (RobotContainer.artifactsInRamp == 9)
            RobotContainer.telemetrySubsystem.ascii_9(TelemetrySubsystem.RCTelemetryMode.OPERATOR);
    }

    // place special subsystem methods here


    /**
     * if the ramp total in RobotContainer is less than 9, increases the ramp total by 1
     */
    public void increaseRampTotal(){
        if (RobotContainer.artifactsInRamp < 9)
            RobotContainer.artifactsInRamp++;
    }

    /**
     * if the ramp total in RobotContainer is greater than 0, decreases the ramp total by 1
     */
    public void decreaseRampTotal(){
        if (RobotContainer.artifactsInRamp > 0)
            RobotContainer.artifactsInRamp--;
    }

    /**
     * resets the ramp total in RobotContainer to 0
     */
    public void resetRampTotal(){
        RobotContainer.artifactsInRamp = 0;
    }
}