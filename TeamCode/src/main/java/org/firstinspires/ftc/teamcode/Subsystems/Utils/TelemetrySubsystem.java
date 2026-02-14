package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.Cameras.LimeLight;

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

    /** what is displayed as part of this telemetry?
     *
     * runs when testing telemetry
     */
    public void testingTelemetryEmpty(){
        if (testingTelemetry){
            //telemetry goes here
        }
    }

    // place special subsystem methods here

    /** what is displayed as part of this telemetry?
     *
     * runs when testing telemetry
     */
    public void ken(){
        if (testingTelemetry){
            addData("LeftPresent", RobotContainer.colour.isLeftArtifactPresent());
            addData("RightPresent", RobotContainer.colour.isRightArtifactPresent());
            addData("LeftColour", RobotContainer.artifactCamera.getRightColour().name());
            addData("RightColour", RobotContainer.artifactCamera.getLeftColour().name());
        }
    }

    /** what is displayed as part of this telemetry?
     *
     * runs when not testing telemetry
     */
    public void operatorTelemetryEmpty(){
        if (!testingTelemetry) {
            //telemetry goes here
        }
    }

    /**
     *Adds an item to the end if the telemetry being built for driver station display. The value shown will be the result of calling toString() on the provided value object. The caption and value are shown on the driver station separated by the #getCaptionValueSeparator() caption value separator. The item is removed if clear() or clearAll() is called.
     * @param caption the caption to use
     * @param value the value to display
     */
    public void addData(String caption, Object value){
        if (testingTelemetry) {
            RCTelemetry.addData(caption, value);
        }
    }

    /**
     *Adds an item to the end if the telemetry being built for driver station display. The value shown will be the result of calling toString() on the provided value object. The caption and value are shown on the driver station separated by the #getCaptionValueSeparator() caption value separator. The item is removed if clear() or clearAll() is called.
     * @param caption the caption to use
     * @param value the value to display
     * @param telemetryForTesting If true the data will be shown with the testing data, if false the data will be shown with the operator data
     */
    public void addData(String caption, Object value, boolean telemetryForTesting){
        if (testingTelemetry == telemetryForTesting) {
            RCTelemetry.addData(caption, value);
        }
    }

    /**
     *Creates and returns a new line in the receiver Telemetry.
     *
     *  @param caption the caption for the line
     */
    public void addLine(String caption){
        if (testingTelemetry) {
            RCTelemetry.addLine(caption);
        }
    }

    /**
     *Creates and returns a new line in the receiver Telemetry.
     *
     *  @param caption the caption for the line
     *  @param telemetryForTesting If true the data will be shown with the testing data, if false the data will be shown with the operator data
     */
    public void addLine(String caption, boolean telemetryForTesting){
        if (testingTelemetry == telemetryForTesting) {
            RCTelemetry.addLine(caption);
        }
    }

    /**
     * displays robot fieldX, fieldY, and Yaw.
     *
     * Runs when testing telemetry.
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

    /**displays ascii art of the number 0
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

    /**displays ascii art of the number 1
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

    /**displays ascii art of the number 2
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

    /**displays ascii art of the number 3
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

    /**displays ascii art of the number 4
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

    /**displays ascii art of the number 5
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

    /**displays ascii art of the number 6
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

    /**displays ascii art of the number 7
     * runs when not testing telemetry
     */
    public void ascii_7(){
        if (!testingTelemetry) {
            RCTelemetry.addLine("■■■■■■■■");
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

    /**displays ascii art of the number 8
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

    /**displays ascii art of the number 9
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


    /**
     * Sends the receiver {@link Telemetry} to the driver station if more than the {@link #getMsTransmissionInterval()
     * transmission interval} has elapsed since the last transmission, or schedules the transmission
     * of the receiver should no subsequent {@link Telemetry} state be scheduled for transmission before
     * the {@link #getMsTransmissionInterval() transmission interval} expires.
     */
    public void update(){
        //ken();
        RCTelemetry.update();
    }


    /**toggles telemetry mode between testing and operator
     *
     */
    public void telemetryModeToggle(){
        testingTelemetry = !testingTelemetry;
    }
}