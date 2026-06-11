package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Place description of subsystem here
 *
 * @author YourGithubName
 */
public class TurretResetButton extends SubsystemBase {

    // Local objects and variables here
    /** The touch sensor object */
    private final TouchSensor resetbutton;

    /** Place code here to initialize subsystem */
    public TurretResetButton() {

        // create touch sensor button
        resetbutton = RobotContainer.ActiveOpMode.hardwareMap.get(TouchSensor.class, "resetTurretButton");
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    /**
     * Returns if button has been pressed
     *
     * @return true if the touch sensor is pressed, false otherwise.
     */
    public boolean hasTouched() {
        return resetbutton.isPressed();
    }
}