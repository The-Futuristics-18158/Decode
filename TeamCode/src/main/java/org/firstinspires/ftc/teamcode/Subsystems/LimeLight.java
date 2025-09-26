package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
/** Gyro Subsystem */
public class LimeLight extends SubsystemBase {

    // robot limelight
    private Limelight3A limeLight;


    /** Place code here to initialize subsystem */
    public LimeLight() { // initialize limelight in
        limeLight = RobotContainer.ActiveOpMode.hardwareMap.get(Limelight3A.class, "limeLight");
        limeLight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limeLight.start(); // This tells Limelight to start looking

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    /**
     * Get gyro angle
     *
     * @return angle in deg between -180 and 180
     */
    public double getYawAngle() {
        return YawAngle;
    }

    /**
     * Resets gyro and offset value
     */
    public void resetYawAngle() {
        setYawAngle(0.0);
    }

    /**
     * sets gyro to provided angle (in deg)
     *
     * @param angle an angle in degrees
     */
    public void setYawAngle(double angle) {

        YawAngleOffset -= getYawAngle() - angle;
    }

}
