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

    public enum RCTelemetryMode {
        OFF,
        TESTING,
        OPERATOR
    }

    public static RCTelemetryMode CurrentTelemetryMode= RCTelemetryMode.TESTING;

    /** Place code here to initialize subsystem */
    public TelemetrySubsystem() {
        RCTelemetry = RobotContainer.ActiveOpMode.telemetry;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    /**used to switch telemetry mode between OFF, TESTING and OPERATOR
     * @param mode the mode to switch to
     */
    public void setRCTelemetryMode(RCTelemetryMode mode){
        CurrentTelemetryMode = mode;
    }



    /** what is displayed as part of this telemetry?
     *
     * runs when testing telemetry
     */
    public void testingTelemetryEmpty(){
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING) {
            //telemetry goes here
        }
    }

    // place special subsystem methods here

    /** what is displayed as part of this telemetry?
     *
     * runs when testing telemetry
     */
    public void ken(){
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING){
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
        if (CurrentTelemetryMode == RCTelemetryMode.OPERATOR) {
            //telemetry goes here
        }
    }

    /**
     *Adds an item to the end if the telemetry being built for driver station display. The value shown will be the result of calling toString() on the provided value object. The caption and value are shown on the driver station separated by the #getCaptionValueSeparator() caption value separator. The item is removed if clear() or clearAll() is called.
     * @param caption the caption to use
     * @param value the value to display
     */
    public void addData(String caption, Object value){
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING) {
            RCTelemetry.addData(caption, value);
        }
    }

    /**
     *Adds an item to the end if the telemetry being built for driver station display. The value shown will be the result of calling toString() on the provided value object. The caption and value are shown on the driver station separated by the #getCaptionValueSeparator() caption value separator. The item is removed if clear() or clearAll() is called.
     * @param caption the caption to use
     * @param value the value to display
     * @param mode sets the mode in which the data will be shown. If the mode is the same as the current telemetry mode, the data will be shown. Otherwise, it will not be shown.
     */
    public void addData(String caption, Object value, RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
            RCTelemetry.addData(caption, value);
        }
    }

    /**
     *Creates and returns a new line in the receiver Telemetry.
     *
     *  @param caption the caption for the line
     */
    public void addLine(String caption){
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING) {
            RCTelemetry.addLine(caption);
        }
    }

    /**
     *Creates and returns a new line in the receiver Telemetry.
     *
     *  @param caption the caption for the line
     *  @param mode sets the mode in which the data will be shown. If the mode is the same as the current telemetry mode, the data will be shown. Otherwise, it will not be shown.
     */
    public void addLine(String caption, RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
            RCTelemetry.addLine(caption);
        }
    }

    /**
     * adds multiple lines of telemetry at once.
     *
     */
    public void addTelemetryBlock(String[] telemetry){
        if(CurrentTelemetryMode == RCTelemetryMode.TESTING) {
            String nextCaption = "";
            for (String line : telemetry) {
                if (line != null && !line.isEmpty()) {
                    if(line.charAt(0) == '!') {
                        RCTelemetry.addLine(line.substring(1));
                    } else if (line.charAt(0) == '@') {
                        nextCaption = line.substring(1);
                    } else {
                        RCTelemetry.addData(nextCaption, line);
                    }
                }
            }
        }
    }

    /**
     * displays robot fieldX, fieldY, and Yaw.
     *
     * Runs when testing telemetry.
     */
    public void odometryTelemetry(){
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING) {
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
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING){
            RCTelemetry.addData("interval time(ms)", RobotContainer.intervaltime);
            RCTelemetry.addData("execute time(ms)", RobotContainer.exectimer.milliseconds());
        }
    }

    /**shows current obelisk ID
     *
     * runs when testing telemetry
     */
    public void currentObeliskId(){
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING){
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
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING) {
            RCTelemetry.addData("Climb distance", RobotContainer.climb.getClimbDistance());
            RCTelemetry.addData("roll", RobotContainer.gyro.getRollAngle());
        }
    }

    /**displays ascii art of the number 0
     * runs when not testing telemetry
     */
    public void ascii_0(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_1(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_2(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_3(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_4(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_5(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_6(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_7(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_8(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
    public void ascii_9(RCTelemetryMode mode){
        if (CurrentTelemetryMode == mode) {
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
        if (CurrentTelemetryMode == RCTelemetryMode.TESTING)
            CurrentTelemetryMode = RCTelemetryMode.OPERATOR;
        else if (CurrentTelemetryMode == RCTelemetryMode.OPERATOR)
            CurrentTelemetryMode = RCTelemetryMode.TESTING;
    }
}