package org.firstinspires.ftc.teamcode.Subsystems.Sensors;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author knutt5
 */
public class GoalTargeting extends SubsystemBase {

    Translation2d redGoalNear = new Translation2d(-1.63, 1.55);
    Translation2d blueGoalNear = new Translation2d (-1.63, -1.55);
    Translation2d redGoalFar = new Translation2d(-1.63, 1.55);
    Translation2d blueGoalFar = new Translation2d(-1.63, -1.55);

    // enable/disable auto target mode
    private boolean AutoTargetingEnable;

    public GoalTargeting()
    {
        // by default, enable auto targeting
        AutoTargetingEnable = true;
    }



    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {


        if (AutoTargetingEnable) {

            // determine if we have target in camera, otherwise use odometry
            LLResult results = RobotContainer.limeLight.getLimeLightResults();
            // do we have valid hits in camera
            boolean CameraHasValidTarget = false;
            double CameraTargetXAngle = 0.0;
            double TargetDistance = 0.0;
            if (results != null && results.isValid() && results.getFiducialResults() != null &&
                    !results.getFiducialResults().isEmpty() && results.getStaleness() < 100)
                for (int i = 0; i < results.getFiducialResults().size(); ++i)
                    if ((RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 24) ||
                            (!RobotContainer.isRedAlliance() && results.getFiducialResults().get(i).getFiducialId() == 20)) {
                        // we have a hit. Record xangle and distance to the valid target
                        CameraHasValidTarget = true;
                        CameraTargetXAngle = results.getFiducialResults().get(i).getTargetXDegrees();
                        Position vector = results.getFiducialResults().get(i).getTargetPoseRobotSpace().getPosition();
                        TargetDistance = Math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
                    }

            // ---------- Hood Angle and Flywheel Speed ----------

            if (!CameraHasValidTarget)
                // camera does not have target, use odometry to set determine distance to target
                TargetDistance = GetDistanceToGoal();

            if (!CameraHasValidTarget)
                RobotContainer.telemetrySubsystem.addData("distance", TargetDistance, true);
            else
                RobotContainer.telemetrySubsystem.addData("distance(AT)", TargetDistance, true);

            RobotContainer.hoodtilt.SetHoodPosition(CalculateHoodAngle(TargetDistance));
            RobotContainer.shooter.SetFlywheelSpeed(CalculateSpeed(TargetDistance));


            // ---------- Turret Angle ----------

            // aiming of turret depends on if we have valid target in camera sight or not
            if (CameraHasValidTarget) {
                // we have target in camera sight - adjust turret by angle to target
                double currentturretangle = RobotContainer.turret.getTurretDegrees();
                RobotContainer.turret.moveTurret(currentturretangle - CameraTargetXAngle);
            } else {
                // use odometry to set turret angle
                Pose2d pose = RobotContainer.odometry.getCurrentPos();
                Translation2d targetPose = GetShotTaget();
                double angle_rad = (new Vector2d(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY())).angle();
                double turretTargetAngle = Math.toDegrees(angle_rad) - 180.0;
                if (RobotContainer.isRedAlliance())
                    turretTargetAngle += 360.0;
                turretTargetAngle -= (RobotContainer.gyro.getYawAngle() + 180.0) % 360.0 - 180.0;
                RobotContainer.turret.moveTurret(turretTargetAngle);
            }
        }

    }

    /**
     * Enables/Disables auto targeting
     */
    public void EnableAutoTargeting(boolean enable)
    {
        AutoTargetingEnable = enable;
        // if switching off enable, turn off shooter
        if (!enable)
            RobotContainer.shooter.SetFlywheelSpeed(0.0);
    }

    public boolean isAutoTargetingEnabled()
    { return AutoTargetingEnable;}


    /* ---------- Left/Right Shoot Solutions ---------- */

    public enum ShootSide {
        LEFT, RIGHT, NONE, BOTH
    }


    /* ---------- Shoot Distance/Speed Calcs ---------- */

    /**add description here
     *
     * @author kaitlyn
     *
     * @return what does this return?
     */
    public double GetDistanceToGoal (){
        Pose2d currentPos = RobotContainer.odometry.getCurrentPos();
        Translation2d goalPose;
        if(RobotContainer.isRedAlliance()){
            if (currentPos.getX() <=1.90)
                goalPose = redGoalNear;
            else
                goalPose = redGoalFar;
        }else{
            if (currentPos.getX() <=1.90)
                goalPose = blueGoalNear;
            else
                goalPose = blueGoalFar;
        }

        double x = goalPose.getX() - currentPos.getX();
        double y = goalPose.getY() - currentPos.getY();

        return Math.sqrt((x*x) + (y*y));
    }


    /**add description here
     *
     * @author kaitlyn
     *
     * @return Returns the speed of the flywheel
     */
    public double CalculateSpeed(double distance){

        // equation from June 04/2026 early PM 8:40pm
        return 426.05 * distance + 1606;
    }

    /**add description here
     *
     * @author superzokabear
     *
     * @return what does this return?
     */
    public double CalculateHoodAngle(double distance){

        // equation from June 04/2026 early PM 8:40pm
        return -0.0096 * distance * distance + 0.0839 * distance - 0.0655;
    }


    /**add description here
     *
     * @author superzokabear
     */
    public Translation2d GetShotTaget(){
        if(RobotContainer.isRedAlliance()){
            return redGoalNear;
        }else{
            return blueGoalNear;
        }
    }



//    /**add description here
//     * @author kaitlyn
//     *
//     * @return what does this return?
//     */
//    public double IdleSpeed(){
//        double shootSpeed = this.CalculateSpeed();
//        double distance = this.GetDistanceToGoal();
////        if (RobotContainer.odometry.getCurrentPos().getX() > (boxPos.getX() - error) &&
////            RobotContainer.odometry.getCurrentPos().getX() < (boxPos.getX() + error) &&
////            RobotContainer.odometry.getCurrentPos().getY() > (boxPos.getY() - error) &&
////            RobotContainer.odometry.getCurrentPos().getY() < (boxPos.getY() + error)) {
////            return 0.0;
//        if(distance >= 2.8) {
//            return (0.7 * shootSpeed);
//        }else {
//            return 0.0;
//        }
//
//    }

}