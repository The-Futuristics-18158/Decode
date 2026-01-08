package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * methods to manage operator controls (gamepads, etc)
 *
 * @author Kw126
 */
public class OperatorControlsSubsystem extends SubsystemBase {

    // Local objects and variables here

    /** Place code here to initialize subsystem */
    public OperatorControlsSubsystem() {

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        if (RobotContainer.artifactsInRamp == 0){
            RobotContainer.RCTelemetry.addLine("◢■■■■◣");
            RobotContainer.RCTelemetry.addLine("◢■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("◥■■■■■■◤");
            RobotContainer.RCTelemetry.addLine("  ◥■■■■◤");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 1){
            RobotContainer.RCTelemetry.addLine("  ◢■■■");
            RobotContainer.RCTelemetry.addLine(" ◢■■■■");
            RobotContainer.RCTelemetry.addLine("      ■■■");
            RobotContainer.RCTelemetry.addLine("      ■■■");
            RobotContainer.RCTelemetry.addLine("      ■■■");
            RobotContainer.RCTelemetry.addLine(" ■■■■■■■");
            RobotContainer.RCTelemetry.addLine(" ■■■■■■■");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 2){
            RobotContainer.RCTelemetry.addLine("  ◢■■■■◣");
            RobotContainer.RCTelemetry.addLine("◢■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■◤    ◥■■");
            RobotContainer.RCTelemetry.addLine("         ◢■◤");
            RobotContainer.RCTelemetry.addLine("      ◢■◤");
            RobotContainer.RCTelemetry.addLine("     ◢■◤");
            RobotContainer.RCTelemetry.addLine("   ◢■◤");
            RobotContainer.RCTelemetry.addLine("◢■■■■■■■");
            RobotContainer.RCTelemetry.addLine("■■■■■■■■");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 3){
            RobotContainer.RCTelemetry.addLine("  ◢■■■■◣");
            RobotContainer.RCTelemetry.addLine("◢■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■◤    ◥■■");
            RobotContainer.RCTelemetry.addLine("         ◢■◤");
            RobotContainer.RCTelemetry.addLine("      ■■■");
            RobotContainer.RCTelemetry.addLine("         ◥■◣");
            RobotContainer.RCTelemetry.addLine("■■◣    ◥■■");
            RobotContainer.RCTelemetry.addLine("◥■■■■■■◤");
            RobotContainer.RCTelemetry.addLine("  ◥■■■■◤");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 4){
            RobotContainer.RCTelemetry.addLine("■■■     ■■■");
            RobotContainer.RCTelemetry.addLine("■■■     ■■■");
            RobotContainer.RCTelemetry.addLine("■■■     ■■■");
            RobotContainer.RCTelemetry.addLine("■■■■■■■■■");
            RobotContainer.RCTelemetry.addLine("■■■■■■■■■");
            RobotContainer.RCTelemetry.addLine("        ■■■");
            RobotContainer.RCTelemetry.addLine("        ■■■");
            RobotContainer.RCTelemetry.addLine("        ■■■");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 5){
            RobotContainer.RCTelemetry.addLine("■■■■■■■■");
            RobotContainer.RCTelemetry.addLine("■■■■■■■■");
            RobotContainer.RCTelemetry.addLine("■■■");
            RobotContainer.RCTelemetry.addLine("■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("        ■■■");
            RobotContainer.RCTelemetry.addLine("        ■■■");
            RobotContainer.RCTelemetry.addLine("■■■■■■■◤");
            RobotContainer.RCTelemetry.addLine("■■■■■■◤");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 6){
            RobotContainer.RCTelemetry.addLine("    ◢■■■■■■");
            RobotContainer.RCTelemetry.addLine("◢■■■■■■■");
            RobotContainer.RCTelemetry.addLine("■■■");
            RobotContainer.RCTelemetry.addLine("■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("◥■■■■■■◤");
            RobotContainer.RCTelemetry.addLine("  ◥■■■■◤");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 7){
            RobotContainer.RCTelemetry.addLine("■■■■■■■■ ");
            RobotContainer.RCTelemetry.addLine("■■■■■■■■");
            RobotContainer.RCTelemetry.addLine("        ■■■");
            RobotContainer.RCTelemetry.addLine("       ◢■■◤");
            RobotContainer.RCTelemetry.addLine("    ◢■■◤");
            RobotContainer.RCTelemetry.addLine("   ◢■■◤");
            RobotContainer.RCTelemetry.addLine(" ◢■■◤");
            RobotContainer.RCTelemetry.addLine("■■◤");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 8){
            RobotContainer.RCTelemetry.addLine("  ◢■■■■◣");
            RobotContainer.RCTelemetry.addLine("◢■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("◥■■■■■■◤");
            RobotContainer.RCTelemetry.addLine("◢■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("◥■■■■■■◤");
            RobotContainer.RCTelemetry.addLine("  ◥■■■■◤");
            RobotContainer.RCTelemetry.addLine("");}
        else if (RobotContainer.artifactsInRamp == 9){
            RobotContainer.RCTelemetry.addLine("  ◢■■■■◣");
            RobotContainer.RCTelemetry.addLine("◢■■■■■■◣");
            RobotContainer.RCTelemetry.addLine("■■■    ■■■");
            RobotContainer.RCTelemetry.addLine("◥■■■■■■■");
            RobotContainer.RCTelemetry.addLine("   ◥■■■■■■");
            RobotContainer.RCTelemetry.addLine("         ■■■");
            RobotContainer.RCTelemetry.addLine("         ■■■");
            RobotContainer.RCTelemetry.addLine("         ■■■");
            RobotContainer.RCTelemetry.addLine("         ■■■");
            RobotContainer.RCTelemetry.addLine("");}
        RobotContainer.RCTelemetry.update();
    }

    // place special subsystem methods here


    /**
     * if the ramp total is less than 9, increase the ramp total by 1
     */
    public void increaseRampTotal(){
        if (RobotContainer.artifactsInRamp < 9)
            RobotContainer.artifactsInRamp++;
    }

    /**
     * if the ramp total is greater than 0, decrease the ramp total by 1
     */
    public void decreaseRampTotal(){
        if (RobotContainer.artifactsInRamp > 0)
            RobotContainer.artifactsInRamp--;
    }

    /**
     * reset the ramp total to 0
     */
    public void resetRampTotal(){
        RobotContainer.artifactsInRamp = 0;
    }
}