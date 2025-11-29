package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
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




    /** Place code here to initialize subsystem */
    public GoalTargeting() {

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
    }

    // returns list of available shooting locations
    public List<Translation2d> GetListOfSHootingPoints() {
        return ShootingCoordinates;
    }



}