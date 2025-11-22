package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotContainer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/** Gyro Subsystem */
public class LimeLight extends SubsystemBase {

    // robot limelight
    private Limelight3A limeLight;
    private Pose2D pose2D;
    public static enum pipeline {
        POSITION(0),
        TAG_GPP(21),
        TAG_PGP(22),
        TAG_PPG(23);
        private final int value;

        pipeline(int value) {
            this.value = value;
        }
    }
    private Telemetry telemetry = RobotContainer.RCTelemetry;
    private LLResult result;
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
        result = limeLight.getLatestResult();
        if (result != null){
            telemetry.addData("Has valid result", result.isValid());
        }else{
            telemetry.addData("null result", true);
        }
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
        result = limeLight.getLatestResult();
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