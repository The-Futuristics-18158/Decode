package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotContainer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/** Gyro Subsystem */
public class LimeLight extends SubsystemBase {

    //  limelight constants
    private Limelight3A limeLight;
    private Pose2D pose2D;

    // tag id for obelisk tags
    public static enum tagId {
        TAG_GPP(21),
        TAG_PGP(22),
        TAG_PPG(23),
        TAG_NULL(0);
        private final int value;

        tagId(int value) {
            this.value = value;
        }
    }
    // different pipelines for obelisk tags and location tags
    public  static enum pipeline{
        OBELISK_MODE,
        POSITION_MODE;
    }

    private Telemetry telemetry = RobotContainer.RCTelemetry;

    private LLResult result;
    /** Place code here to initialize subsystem */
    public LimeLight() { // initialize limelight in
        limeLight = RobotContainer.ActiveOpMode.hardwareMap.get(Limelight3A.class, "limeLight");
        limeLight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limeLight.start(); // This tells Limelight to start looking
        limeLight.pipelineSwitch(1);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        // tell Limelight which way robot is facing
        double robotYaw = RobotContainer.gyro.getYawAngle();
        limeLight.updateRobotOrientation(robotYaw);
        result = limeLight.getLatestResult();

//       telemetry.addData("Obelisk ID", getObeliskID());;
        LLResultTypes.FiducialResult kaitlyn = getTargetInfo();
        if (kaitlyn != null){
            telemetry.addData("target angle", kaitlyn.getTargetXDegrees());
        }else{
            telemetry.addData("no target", 0);
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

    // switch between detecting obelisk id and position tags
    public void SetPipelineMode(int pipelineMode){
        if (pipelineMode == 0 || pipelineMode == 1){
            limeLight.pipelineSwitch(pipelineMode);
        }
    }
    // get obelisk id from limelight
    public tagId getObeliskID(){
        int detectedTag = tagId.TAG_NULL.value;
        LLResult detectedtags=limeLight.getLatestResult();
        List<LLResultTypes.FiducialResult> results = detectedtags.getFiducialResults();

        if (results!=null && !results.isEmpty()){
            detectedTag = results.get(0).getFiducialId();

            if (detectedTag == tagId.TAG_GPP.value){
                return tagId.TAG_GPP;
            } else if (detectedTag == tagId.TAG_PPG.value ) {
                return tagId.TAG_PPG;
            } else if (detectedTag == tagId.TAG_PGP.value) {
                return tagId.TAG_PGP;
            }
        }
        return tagId.TAG_NULL;
    }

    public LLResultTypes.FiducialResult getTargetInfo(){
        LLResult detectedtags = limeLight.getLatestResult();
        List<LLResultTypes.FiducialResult> results = detectedtags.getFiducialResults();
        if (results!=null){
            if(RobotContainer.isRedAlliance() && results.size()>0 && results.get(0).getFiducialId() == 24){
                return results.get(0);
            } else if (RobotContainer.isRedAlliance() && results.size()>1 && results.get(1).getFiducialId() == 24) {
                return results.get(1);
            }else if (!RobotContainer.isRedAlliance() && results.size()>0 && results.get(0).getFiducialId() == 20){
                return  results.get(0);
            }else if (!RobotContainer.isRedAlliance() && results.size()>1 && results.get(1).getFiducialId() == 20){
                return results.get(1);
            }else{
                return  null;
            }

        }else{
            return null;
        }



    }

}