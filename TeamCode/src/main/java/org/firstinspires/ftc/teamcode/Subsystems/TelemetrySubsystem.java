package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.SensorsAndCameras.LimeLight;

/**
 * Subsystem to manage telemetry
 *
 * @author Kw126
 */
public class TelemetrySubsystem extends SubsystemBase {

    // Local objects and variables here
    public static Telemetry RCTelemetry;
    public static boolean testingTelemetry = true;

    /** Place code here to initialize subsystem */
    public TelemetrySubsystem() {
        RCTelemetry = RobotContainer.ActiveOpMode.telemetry;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here

    /**
     *
     * runs when testing telemetry
     */
    public void testingTelemetryEmpty(){
        if (testingTelemetry){

        }
    }

    /**
     *
     * runs when not testing telemetry
     */
    public void operatorTelemetryEmpty(){
        if (!testingTelemetry) {

        }
    }

    public void addData(String caption, Object value, boolean telemetryForTesting){
        if (testingTelemetry == telemetryForTesting) {
            RCTelemetry.addData(caption, value);
        }
    }

    public void addLine(String caption, boolean telemetryForTesting){
        if (testingTelemetry == telemetryForTesting) {
            RCTelemetry.addLine(caption);
        }
    }

    /**displays robot fieldX, fieldY, and Yaw
     *
     * runs when testing telemetry
     */
    public void odometryTelemetry(){
        if (testingTelemetry) {
            Pose2d position = RobotContainer.odometry.getCurrentPos();
            RCTelemetry.addData("fieldX", position.getX());
            RCTelemetry.addData("fieldY", position.getY());
            RCTelemetry.addData("Yaw", position.getRotation().getDegrees());
        }
    }

    /**displays interval time and execute time
     *
     * runs when testing telemetry
     */
    public void timerOdometry(){
        if (testingTelemetry){
            RCTelemetry.addData("interval time(ms)", RobotContainer.intervaltime);
            RCTelemetry.addData("execute time(ms)", RobotContainer.exectimer.milliseconds());
        }
    }

    /**shows current obelisk ID
     *
     * runs when testing telemetry
     */
    public void currentObeliskId(){
        if (testingTelemetry){
            if (RobotContainer.GetCurrentMode()== RobotContainer.Modes.AutoInit) {
                if (RobotContainer.limeLight.getObeliskID()== LimeLight.tagId.TAG_GPP)
                    RCTelemetry.addLine("See GPP Tag");
                else if (RobotContainer.limeLight.getObeliskID()== LimeLight.tagId.TAG_PGP)
                    RCTelemetry.addLine("See PGP Tag");
                else if (RobotContainer.limeLight.getObeliskID()== LimeLight.tagId.TAG_PPG)
                    RCTelemetry.addLine("See PPG Tag");
                else
                    RCTelemetry.addLine("No Obelisk Tag");
            }
        }
    }

    /**shows climbDistance and roll
     *
     * runs when testing telemetry
     */
    public void climbTelemetry(){
        if (testingTelemetry) {
            RCTelemetry.addData("Climb distance", RobotContainer.climb.getClimbDistance());
            RCTelemetry.addData("roll", RobotContainer.gyro.getRollAngle());
        }
    }

    /**displays ascii art of the number 0*
     * runs when not testing telemetry
     */
    public void ascii_0(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("□◥■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 1*
     * runs when not testing telemetry
     */
    public void ascii_1(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■□□");
            RCTelemetry.addLine("◢■■■■□□");
            RCTelemetry.addLine("□□■■■□□");
            RCTelemetry.addLine("□□■■■□□");
            RCTelemetry.addLine("□□■■■□□");
            RCTelemetry.addLine("□□■■■□□");
            RCTelemetry.addLine("■■■■■■■");
            RCTelemetry.addLine("■■■■■■■");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 2*
     * runs when not testing telemetry
     */
    public void ascii_2(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■◤□□◥■■");
            RCTelemetry.addLine("□□□□□◢■◤");
            RCTelemetry.addLine("□□□□◢■◤□");
            RCTelemetry.addLine("□□□◢■◤□□");
            RCTelemetry.addLine("□□◢■◤□□□");
            RCTelemetry.addLine("◢■■■■■■■");
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 3*
     * runs when not testing telemetry
     */
    public void ascii_3(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■◤□□◥■■");
            RCTelemetry.addLine("□□□□□◢■◤");
            RCTelemetry.addLine("□□□□■■■□");
            RCTelemetry.addLine("□□□□□◥■◣");
            RCTelemetry.addLine("■■◣□□◢■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("□◥■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 4*
     * runs when not testing telemetry
     */
    public void ascii_4(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("■■■□□■■■□");
            RCTelemetry.addLine("■■■□□■■■□");
            RCTelemetry.addLine("■■■□□■■■□");
            RCTelemetry.addLine("■■■■■■■■■");
            RCTelemetry.addLine("■■■■■■■■■");
            RCTelemetry.addLine("□□□□□■■■□");
            RCTelemetry.addLine("□□□□□■■■□");
            RCTelemetry.addLine("□□□□□■■■□");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 5*
     * runs when not testing telemetry
     */
    public void ascii_5(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("■■■□□□□□");
            RCTelemetry.addLine("■■■■■■◣□");
            RCTelemetry.addLine("■■■■■■■◣");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("■■■■■■■◤");
            RCTelemetry.addLine("■■■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 6*
     * runs when not testing telemetry
     */
    public void ascii_6(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■■■");
            RCTelemetry.addLine("◢■■■■■■■");
            RCTelemetry.addLine("■■■□□□□□");
            RCTelemetry.addLine("■■■■■■◣□");
            RCTelemetry.addLine("■■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("□◥■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 7*
     * runs when not testing telemetry
     */
    public void ascii_7(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("■■■■■■■■ ");
            RCTelemetry.addLine("■■■■■■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□◢■■◤");
            RCTelemetry.addLine("□□□◢■■◤□");
            RCTelemetry.addLine("□□◢■■◤□□");
            RCTelemetry.addLine("□◢■■◤□□□");
            RCTelemetry.addLine("◢■■◤□□□□");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 8*
     * runs when not testing telemetry
     */
    public void ascii_8(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■◤");
            RCTelemetry.addLine("□◥■■■■◤□");
            RCTelemetry.addLine("");
        }
    }

    /**displays ascii art of the number 9*
     * runs when not testing telemetry
     */
    public void ascii_9(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("□◢■■■■◣□");
            RCTelemetry.addLine("◢■■■■■■◣");
            RCTelemetry.addLine("■■■□□■■■");
            RCTelemetry.addLine("◥■■■■■■■");
            RCTelemetry.addLine("□◥■■■■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("□□□□□■■■");
            RCTelemetry.addLine("");
        }
    }

    public void update(){
        RCTelemetry.update();
    }


    /**toggles telemetry mode between testing and operator
     *
     */
    public void telemetryModeToggle(){
        testingTelemetry = !testingTelemetry;
    }
}