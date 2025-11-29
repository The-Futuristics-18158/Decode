package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.util.ArrayList;
import java.util.List;

/**
 * Obelisk module is used to monitor and record obelisk pattern.
 * Pattern is saved and can be referred to throughout game
 * Module also contains functions to return 1st, 2nd and 3rd colour in obelisk pattern
 *
 * @author knutt5
 */
public class Obelisk extends SubsystemBase {

    // the stored obelisk pattern for the game
    private static ObeliskPattern pattern = null;

    // pattern that we are currently looking at but not yet finalized
    private ObeliskPattern pattern_temp;

    public enum ArtifactColor {
       Purple,
       Green }
    public enum ObeliskPattern {
        GPP,
        PGP,
        PPG };

    /** Place code here to initialize subsystem */
    public Obelisk() {

        // we don't have pattern by default
        pattern_temp = null;

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // are we currently looking at a tag?
        LimeLight.tagId tag = RobotContainer.limeLight.getObeliskID();

        // if we are looking at a tag, then record it
        if (tag == LimeLight.tagId.TAG_GPP)
            pattern_temp = ObeliskPattern.GPP;
        if (tag == LimeLight.tagId.TAG_PGP)
            pattern_temp = ObeliskPattern.PGP;
        if (tag == LimeLight.tagId.TAG_PPG)
            pattern_temp = ObeliskPattern.PPG;

        if (pattern!=null)
        {
            if (pattern==ObeliskPattern.GPP)
                RobotContainer.RCTelemetry.addLine("Pattern GPP");
            if (pattern==ObeliskPattern.PGP)
                RobotContainer.RCTelemetry.addLine("Pattern PGP");
            if (pattern==ObeliskPattern.PPG)
                RobotContainer.RCTelemetry.addLine("Pattern PPG");
        }
    }

    /**
     * Call to record pattern of obelisk
     */
    public void RecordPattern()  {
        if (pattern_temp!=null)
            pattern = pattern_temp;
        else
            // we haven't seen a pattern - use this as default
            pattern = ObeliskPattern.GPP;
    }

    /**
     * Switches camera to obelisk mode
     */
    public void StartObeliskScan() {
        RobotContainer.limeLight.SetPipelineMode(1);
    }

    /**
     * Returns recorded obelisk pattern
     */
    public ObeliskPattern GetPattern() {
        return pattern;
    }


    /**
     * Returns list of colors in obelisk pattern
     * @param index Color index from 0 to 2. Any other value returns purple by default
     */
    public ArtifactColor GetColorAtIndex(int index)
    {
        if ((pattern==ObeliskPattern.GPP && index==0) ||
                (pattern==ObeliskPattern.PGP && index==1) ||
                (pattern==ObeliskPattern.PGP && index==2))
            return ArtifactColor.Green;
        else
            return ArtifactColor.Purple;
    }



}