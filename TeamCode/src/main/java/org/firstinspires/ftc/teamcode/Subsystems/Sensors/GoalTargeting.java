package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Cameras.ArtifactCamera;
import org.firstinspires.ftc.teamcode.Utility.AutoFunctions;
import java.util.List;

/**
 * Place description of subsystem here
 *
 * @author knutt5
 */
public class GoalTargeting extends SubsystemBase {

    // List of all available locations to shoot from
    // specify for blue side. RedVsBlue translates as necessary
    private List <Translation2d> ShootingCoordinates = List.of(
            AutoFunctions.redVsBlue(new Translation2d(0,0)),
            AutoFunctions.redVsBlue(new Translation2d(1.0, 0)),
            AutoFunctions.redVsBlue(new Translation2d(-1.0, -1.0))
            // whatever
    );

    Pose2d redGoalNear = new Pose2d(-1.63, 1.55, new Rotation2d(0));
    Pose2d blueGoalNear = new Pose2d(-1.63, -1.55, new Rotation2d(0));
    Pose2d redGoalFar = new Pose2d(-1.63, 1.45, new Rotation2d(0));
    Pose2d blueGoalFar = new Pose2d(-1.63, -1.45, new Rotation2d(0));


    Pose2d currentPos = new Pose2d();

    Pose2d boxPos;
    double error;

    /** Place code here to initialize subsystem */
    public GoalTargeting() {

        boxPos = AutoFunctions.redVsBlue( new Pose2d(new Translation2d( 0.935, 0.81), new Rotation2d(0)));
        error = 0.40;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        double distance = GetDistanceToGoal();
        RobotContainer.telemetrySubsystem.addData("distance", distance, true);
    }

    // returns list of available shooting locations
    public List<Translation2d> GetListOfSHootingPoints() {
        return ShootingCoordinates;
    }


    /* ---------- Left/Right Shoot Solutions ---------- */

    public enum ShootSide {
        LEFT, RIGHT, NONE, BOTH
    }

    // 2. The Functional Interface (must have one abstract method)
    /** add description here
     *
     * @author kaitlyn
     */
    @FunctionalInterface
    public interface LeftVsRight {
        ShootSide getSide();
    }

    /**Finds any artifact in the robot to shoot
     *
     * @author kaitlyn
     *
     * @return The side of the uptake the artifact to shoot is on
     */
    public LeftVsRight ShootAny()
    {
        // this is lamda function
        return ()-> {
            // Logic to shoot ball
            if(RobotContainer.artifactCamera.IsRightPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.artifactCamera.IsLeftPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    /**Finds a green artifact in the robot to shoot
     *
     * @author kaitlyn
     *
     * @return The side of the uptake the artifact to shoot is on
     */
    public LeftVsRight ShootGreen()
    {
        // this is lamda function
        return ()-> {
            // Logic to shoot ball
            if(RobotContainer.artifactCamera.getRightColour().name().equals(ArtifactCamera.ArtifactColours.Green.name()))
                return ShootSide.LEFT;
            else if (RobotContainer.artifactCamera.getLeftColour().name().equals(ArtifactCamera.ArtifactColours.Green.name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.artifactCamera.IsRightPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.artifactCamera.IsLeftPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    /**Finds a purple artifact in the robot to shoot
     *
     * @author kaitlyn
     *
     * @return The side of the robot the artifact to shoot is on
     */
    public LeftVsRight ShootPurple() {
        // this is lamda function
        return ()-> {
            if (RobotContainer.artifactCamera.getRightColour().name().equals(ArtifactCamera.ArtifactColours.Purple.name()))
                return ShootSide.LEFT;
            else if (RobotContainer.artifactCamera.getLeftColour().name().equals(ArtifactCamera.ArtifactColours.Purple.name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.artifactCamera.IsRightPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.artifactCamera.IsLeftPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    /**add description here
     *
     * @author kaitlyn
     *
     * @return what does this return?
     */
    public LeftVsRight ShootObelisk1() {
        // this is lamda function
        return ()-> {
            // get obelisk color for artifact #1
            Obelisk.ArtifactColor color = RobotContainer.obelisk.GetColorAtIndex(0);
            // Logic to shoot ball
            if((color.name().equals(RobotContainer.artifactCamera.getRightColour().name())))
                return ShootSide.LEFT;
            else if (color.name().equals(RobotContainer.artifactCamera.getLeftColour().name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.artifactCamera.IsRightPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.artifactCamera.IsLeftPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    /**add description here
     *
     * @author kaitlyn
     *
     * @return what does this return?
     */
    public LeftVsRight ShootObelisk2() {
        // this is lamda function
        return ()-> {
            // get obelisk color for artifact #1
            Obelisk.ArtifactColor color = RobotContainer.obelisk.GetColorAtIndex(1);
            // Logic to shoot ball
            if((color.name().equals(RobotContainer.artifactCamera.getRightColour().name())))
                return ShootSide.LEFT;
            else if (color.name().equals(RobotContainer.artifactCamera.getLeftColour().name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.artifactCamera.IsRightPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.artifactCamera.IsLeftPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    /**add description here
     *
     * @author kaitlyn
     *
     * @return what does this return?
     */
    public LeftVsRight ShootObelisk3() {
        // this is lamda function
        return ()-> {
            // get obelisk color for artifact #2
            Obelisk.ArtifactColor color = RobotContainer.obelisk.GetColorAtIndex(2);
            // Logic to shoot ball
            if((color.name().equals(RobotContainer.artifactCamera.getRightColour().name())))
                return ShootSide.LEFT;
            else if (color.name().equals(RobotContainer.artifactCamera.getLeftColour().name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.artifactCamera.IsRightPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.artifactCamera.IsLeftPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }


    /* ---------- Shoot Distance/Speed Calcs ---------- */

    /**add description here
     *
     * @author kaitlyn
     *
     * @return what does this return?
     */
    public double GetDistanceToGoal (){
        currentPos = RobotContainer.odometry.getCurrentPos();
        Pose2d goalPose = new Pose2d();
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

        double hypotenuse = Math.sqrt((x*x) + (y*y));
        return hypotenuse;

    }


    /**add description here
     *
     * @author kaitlyn
     *
     * @return Returns the speed of the flywheel
     */
    public double CalculateSpeed(){
        double x = this.GetDistanceToGoal();

        if (x <= 2.5){
            return 253.51*x + 2403.8;
        }else {
            return 3350;
        }

    }

    /**add description here
     *
     * @author superzokabear
     *
     * @return what does this return?
     */
    public double CalculateHoodAngle(){

        double x = this.GetDistanceToGoal();

        if (x <= 2.5){
            //return -0.6343*x*x + 2.3894*x - 1.8076;
            return -0.6077*x*x + 2.2606*x - 1.6945;
        }else {
            return 0.44;
        }
    }

    public void SetHoodAngleAndSpeed(){
        RobotContainer.hoodtilt.SetHoodPosition(CalculateHoodAngle());
        RobotContainer.shooter.SetFlywheelSpeed(CalculateSpeed());
    }


    /**add description here
     *
     * @author kaitlyn
     *
     * @return what does this return?
     */
    public double IdleSpeed(){
        double shootSpeed = this.CalculateSpeed();
        double distance = this.GetDistanceToGoal();
        if (RobotContainer.odometry.getCurrentPos().getX() > (boxPos.getX() - error) &&
            RobotContainer.odometry.getCurrentPos().getX() < (boxPos.getX() + error) &&
            RobotContainer.odometry.getCurrentPos().getY() > (boxPos.getY() - error) &&
            RobotContainer.odometry.getCurrentPos().getY() < (boxPos.getY() + error)) {
            return 0.0;
        } else if(distance >= 3.2 && distance <= 3.7) {
            return (0.7 * shootSpeed);
        }else {
            return 0.0;
        }

        //if (distance <= 2.0){
        //    return shootSpeed;
        //}else if(distance >= 3.2 && distance <= 3.7){
        //    return (0.7 * shootSpeed);
        //}else {
        //    return 800.0;
        //}
    }

}