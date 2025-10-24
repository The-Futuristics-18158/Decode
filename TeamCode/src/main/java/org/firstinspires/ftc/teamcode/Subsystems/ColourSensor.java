package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.RobotContainer;



/**
 * Place description of subsystem here
 * leftFrontDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
 * @author Blackthrush
 */
public class ColourSensor extends SubsystemBase {

    // Local objects and variables here
    private NormalizedColorSensor sensor;

    /** Place code here to initialize subsystem */
    public ColourSensor() {
    sensor = RobotContainer.ActiveOpMode.hardwareMap.get(NormalizedColorSensor.class, "colour_sensor");
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        NormalizedRGBA my_colour;
        my_colour = sensor.getNormalizedColors();
        RobotContainer.DBTelemetry.addData("Red", "%.3f", my_colour.red);
        RobotContainer.DBTelemetry.addData("Green", "%.3f", my_colour.green);
        RobotContainer.DBTelemetry.addData("Blue", "%.3f", my_colour.blue);
        RobotContainer.DBTelemetry.addData("LONAN", "%.3f", my_colour.alpha);



        if (my_colour.alpha > 0.5){
            RobotContainer.DBTelemetry.addData("ArtictDected", true);

        }
        else{
            RobotContainer.DBTelemetry.addData("ArtictDected", false);
        }
        RobotContainer.DBTelemetry.update();
    }

    // place special subsystem methods here

}