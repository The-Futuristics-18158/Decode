package org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

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



    /** Place code here to initialize subsystem */
    public GoalTargeting() {

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        double distance = GetDistanceToGoal();
        RobotContainer.RCTelemetry.addData("distance", distance);
    }

    // returns list of available shooting locations
    public List<Translation2d> GetListOfSHootingPoints() {
        return ShootingCoordinates;
    }

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
        //double speed = (207.96*(x*x)) - (480.12*(x)) + 2859.6; kaitlyn's
        double speed = (190.58*(x*x)) - (386.65*(x)) + 2741;
        return speed;
    }


}