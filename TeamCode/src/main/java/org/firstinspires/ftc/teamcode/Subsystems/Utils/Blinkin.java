package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Runs the BLinkin LEDs
 *
 * @author YourGithubName
 */
public class Blinkin extends SubsystemBase {
    private RevBlinkinLedDriver blinkin;

    // Local objects and variables here

    /** Place code here to initialize subsystem */
    public Blinkin() {
        blinkin = RobotContainer.ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        if(RobotContainer.isRedAlliance()==true) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

    }
}