package org.firstinspires.ftc.teamcode.Commands.Odomeetry;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;


// command template
public class ResetOdometryXYAngle extends CommandBase {

    boolean FirstSample;
    double PoseX, PoseY, PoseRad;
    int NumberSamples;


    // constructor
    public ResetOdometryXYAngle() {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // do not move while getting odometry position
        RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0);

        // assume first sample is our first reading
        FirstSample=true;

        // reset poses
        PoseX =0.0; PoseY = 0.0; PoseRad = 0.0;

        NumberSamples = 0;
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get limelight MT2 odometry
        LLResult result = RobotContainer.limeLight.limeLight.getLatestResult();

        // if we have valid result and it is not stale (>100ms)
        // and we have at least one apriltag detection
        if (result!=null && result.isValid() && result.getStaleness() < 100 &&
            result.getFiducialResults()!=null && !result.getFiducialResults().isEmpty())
        {
            NumberSamples++;

            // is this our first reading?
            if (FirstSample)
            {
                // our first sample, just record value
               PoseX = result.getBotpose().getPosition().x;
               PoseY = result.getBotpose().getPosition().y;
               PoseRad = result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS);
               FirstSample=false;
            }
            else
            {
                // this is our 2nd or subsequent sample
                // Average in the sample
                PoseX = 0.85*PoseX + 0.15*result.getBotpose().getPosition().x;
                PoseY = 0.85*PoseY + 0.15*result.getBotpose().getPosition().y;
                PoseRad = 0.85*PoseRad + 0.15*result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS);
            }
        }
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        // we are finished if we have had > 4 samples
        return (NumberSamples > 15);

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        // if we have at least one sample, go and set robots current pose
        if (NumberSamples > 0) {
            // set robot's current Pose
            Pose2d CurrentPose = new Pose2d(PoseX, PoseY, new Rotation2d(PoseRad));
            RobotContainer.odometry.setCurrentPos(CurrentPose);
        }
    }

}