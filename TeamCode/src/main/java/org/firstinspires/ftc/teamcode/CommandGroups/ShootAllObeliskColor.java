package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.ShootObeliskColor;

// shoot all artifacts in order of obelisk pattern (if possible)
// typically used in auto
public class ShootAllObeliskColor extends SequentialCommandGroup {

    // constructor
    public ShootAllObeliskColor() {

        addCommands (
                // Student to complete

                // shoot artifact with color index=0
                new ShootObeliskColor(0)
                // intake another artifact
                // shoot again
                // shoot again
                //
                // note: be sure to add inter-step pauses as needed
        );
    }

}

