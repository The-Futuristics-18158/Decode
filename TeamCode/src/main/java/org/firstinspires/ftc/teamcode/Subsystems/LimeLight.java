package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotContainer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PWMOutput;

/** Gyro Subsystem */
public class LimeLight extends SubsystemBase {

    // robot limelight
    private Limelight3A limeLight;
    private Pose2D pose2D;
    public static enum pipeline {
        POSITION,
        TAG_GPP,
        TAG_PGP,
        TAG_PPG
    }


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
        // First, tell Limelight which way your robot is facing
        double robotYaw = RobotContainer.gyro.getYawAngle();
        limeLight.updateRobotOrientation(robotYaw);
        LLResult result = limeLight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                pose2D = new Pose2D(DistanceUnit.METER, x,y,AngleUnit.DEGREES,robotYaw);
            }
        }
    }

    /**
     * Get position from limelight
     *
     * @return Pose2D
     */
    public Pose2D getFieldPosition() {

        return pose2D;
    }

    public boolean hasObelisk() {
        boolean obelisk = false;
        LLResult result = limeLight.getLatestResult();
        limeLight.pipelineSwitch(pipeline.TAG_GPP.ordinal());
        if (result != null && result.isValid()) {
            obelisk = true;
        } else {
            limeLight.pipelineSwitch(pipeline.TAG_PGP.ordinal());
            if (result != null && result.isValid()) {
                obelisk = true;
            } else {
                limeLight.pipelineSwitch(pipeline.TAG_PPG.ordinal());
                if (result != null && result.isValid()) {
                    obelisk = true;
                }
            }

        }


        return obelisk;
    }

    public pipeline getObeliskID(){
        pipeline obelisk = null;
        LLResult result = limeLight.getLatestResult();
        limeLight.pipelineSwitch(pipeline.TAG_GPP.ordinal());
        if (result != null && result.isValid()) {
            obelisk = pipeline.TAG_GPP;
        } else {
            limeLight.pipelineSwitch(pipeline.TAG_PGP.ordinal());
            if (result != null && result.isValid()) {
                obelisk = pipeline.TAG_PGP;
            } else {
                limeLight.pipelineSwitch(pipeline.TAG_PPG.ordinal());
                if (result != null && result.isValid()) {
                    obelisk = pipeline.TAG_PPG;
                }
            }

        }
        return obelisk;
    }




}
