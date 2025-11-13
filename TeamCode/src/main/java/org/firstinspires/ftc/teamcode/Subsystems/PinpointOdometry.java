package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * The Pinpoint Odometry class is a subsystem that manages the odometry pods using the Pinpoint Odometry Computer hardware.
 * It extends the SubsystemBase class from FTCLib.
 */
public class PinpointOdometry extends SubsystemBase {

    // Odometry pods - constants
    private double wheelRadius_mm;
    private double wheelCircumference_mm = Math.PI * 2 * wheelRadius_mm;
    private double ticsPerRev;
    // Travel distance per encoder pulse (m)
    private double tics_per_mm = ticsPerRev / wheelCircumference_mm;
    public double xPos;
    public double yPos;
    // Create pinpoint driver object
    GoBildaPinpointDriver pinpointDriver;


    // Encoder values from each odometry pod
    double OdometryLeftEncoder;
    double OdometryRightEncoder;
    double OdometryFrontEncoder;

    /**
     * Constructor for the OctQuad class.
     * Initializes the OctoQuad hardware and resets encoders.
     */
    public PinpointOdometry() {
        // Set up pinpoint driver object
        pinpointDriver = RobotContainer.ActiveOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint Driver");

        // Reset pinpoint driver
        pinpointDriver.resetPosAndIMU();

        // Set encoder directions
       pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Set encoder resolution
        pinpointDriver.setEncoderResolution(tics_per_mm, DistanceUnit.MM);

    }

    /**
     * The periodic method is called periodically by the scheduler.
     * Reads the encoder values from the odometry pods.
     */
    @Override
    public void periodic() {
        pinpointDriver.update(); // the fast one

    }



}