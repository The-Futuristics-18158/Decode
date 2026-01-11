package org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleLeftUptake;
import org.firstinspires.ftc.teamcode.CommandGroups.Uptake.CycleRightUptake;
import org.firstinspires.ftc.teamcode.RobotContainer;
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

    Pose2d redGoal = new Pose2d(-1.63, 1.63, new Rotation2d(0));
    Pose2d blueGoal = new Pose2d(-1.63, -1.63, new Rotation2d(0));

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
        //RobotContainer.telemetrySubsystem.addData("distance", distance);
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
    @FunctionalInterface
    public interface LeftVsRight {
        ShootSide getSide();
    }

    public LeftVsRight ShootAny()
    {
        // this is lamda function
        return ()-> {
            // Logic to shoot ball
            if(RobotContainer.colour.isLeftArtifactPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.colour.isRightArtifactPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    public LeftVsRight ShootGreen()
    {
        // this is lamda function
        return ()-> {
            // Logic to shoot ball
            if(RobotContainer.colour.GetLeftColour().name().equals(ColourSensor.ArtifactColours.Green.name()))
                return ShootSide.LEFT;
            else if (RobotContainer.colour.GetRightColour().name().equals(ColourSensor.ArtifactColours.Green.name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.colour.isLeftArtifactPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.colour.isRightArtifactPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    public LeftVsRight ShootPurple() {
        // this is lamda function
        return ()-> {
            if (RobotContainer.colour.GetLeftColour().name().equals(ColourSensor.ArtifactColours.Purple.name()))
                return ShootSide.LEFT;
            else if (RobotContainer.colour.GetRightColour().name().equals(ColourSensor.ArtifactColours.Purple.name()))
                return ShootSide.RIGHT;
            else if (RobotContainer.colour.isLeftArtifactPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.colour.isRightArtifactPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    public LeftVsRight ShootObelisk1() {
        // this is lamda function
        return ()-> {
            // get obelisk color for artifact #1
            Obelisk.ArtifactColor color = RobotContainer.obelisk.GetColorAtIndex(0);
            // Logic to shoot ball
            if((color.name().equals(RobotContainer.colour.GetLeftColour().name())))
                return ShootSide.LEFT;
            else if (color.name().equals(RobotContainer.colour.GetRightColour().name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.colour.isLeftArtifactPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.colour.isRightArtifactPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    public LeftVsRight ShootObelisk2() {
        // this is lamda function
        return ()-> {
            // get obelisk color for artifact #1
            Obelisk.ArtifactColor color = RobotContainer.obelisk.GetColorAtIndex(1);
            // Logic to shoot ball
            if((color.name().equals(RobotContainer.colour.GetLeftColour().name())))
                return ShootSide.LEFT;
            else if (color.name().equals(RobotContainer.colour.GetRightColour().name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.colour.isLeftArtifactPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.colour.isRightArtifactPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }

    public LeftVsRight ShootObelisk3() {
        // this is lamda function
        return ()-> {
            // get obelisk color for artifact #2
            Obelisk.ArtifactColor color = RobotContainer.obelisk.GetColorAtIndex(2);
            // Logic to shoot ball
            if((color.name().equals(RobotContainer.colour.GetLeftColour().name())))
                return ShootSide.LEFT;
            else if (color.name().equals(RobotContainer.colour.GetRightColour().name()))
                return ShootSide.RIGHT;
            else if(RobotContainer.colour.isLeftArtifactPresent())
                return ShootSide.LEFT;
            else if (RobotContainer.colour.isRightArtifactPresent())
                return ShootSide.RIGHT;
            else
                return ShootSide.BOTH;
        };
    }


    /* ---------- Shoot Distance/Speed Calcs ---------- */

    public double GetDistanceToGoal (){
        currentPos = RobotContainer.odometry.getCurrentPos();
        Pose2d goalPose = new Pose2d();
        if(RobotContainer.isRedAlliance()){
            goalPose = redGoal;
        }else{
            goalPose = blueGoal;
        }

        double x = goalPose.getX() - currentPos.getX();
        double y = goalPose.getY() - currentPos.getY();

        double hypotenuse = Math.sqrt((x*x) + (y*y));
        return hypotenuse;

    }

    public double CalculateSpeed(){
        double x = this.GetDistanceToGoal();
        //double speed = (207.96*(x*x)) - (480.12*(x)) + 2859.6; kaitlyns
        double speed = (190.58*(x*x)) - (386.65*(x)) + 2741;
        return speed;
    }

    public double CalculateHoodAngle(boolean isLeft){
       double maxAngle;
       double minAnle;

        if (isLeft){
           maxAngle = RobotContainer.hoodtilt.MaxLeftAngle;
            minAnle = RobotContainer.hoodtilt.MinLeftAngle;
        }else{
           maxAngle = RobotContainer.hoodtilt.MaxRightAngle;
           minAnle = RobotContainer.hoodtilt.MinRightAngle;
        }

        double range = maxAngle - minAnle;
        double distance = this.GetDistanceToGoal();
        double angle = (distance/3.4* range) + minAnle;
        return angle;
    }


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