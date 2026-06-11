package org.firstinspires.ftc.teamcode.Subsystems.Cameras;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.RobotContainer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;


/** Gyro Subsystem */
public class LimeLight extends SubsystemBase {

    //  limelight constants
    private Limelight3A limeLight;


    /** Place code here to initialize subsystem */
    public LimeLight() { // initialize limelight in
        limeLight = RobotContainer.ActiveOpMode.hardwareMap.get(Limelight3A.class, "limeLight");
        limeLight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limeLight.start(); // This tells Limelight to start looking
        limeLight.pipelineSwitch(0);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        // tell Limelight which way robot is facing
        //double robotYaw = RobotContainer.gyro.getYawAngle();
        //limeLight.updateRobotOrientation(robotYaw);
    }


    /**Switch between detecting obelisk id and position tags
     * @param pipelineMode an integer representing the desired pipeline mode: 0 for obelisk id detection, 1 for position tag detection, 2 for driver camera mode
     */
    public void SetPipelineMode(int pipelineMode){
        if (pipelineMode == 0 || pipelineMode == 1 || pipelineMode == 2){
            limeLight.pipelineSwitch(pipelineMode);
        }
    }

    /**add description here
     * @return what does this return?
     */
    public LLResult getLimeLightResults() {
        return limeLight.getLatestResult();
    }

}