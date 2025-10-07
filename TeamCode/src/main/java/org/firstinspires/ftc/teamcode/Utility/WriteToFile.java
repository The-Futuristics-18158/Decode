package org.firstinspires.ftc.teamcode.Utility;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 *
 *
 * @author kw126
 */
public class WriteToFile {

    /**
     * Writes a given string to a specified file on the Robot Controller.
     * If the file does not exist, it will be created in the FIRST/settings folder.
     *
     * @author FIRST
     *
     * @param myNumber The double data to be written to the file.
     * @param toFileName The name of the file to write to (e.g., "mydata.txt").
     */
    public static void writeToFile (double myNumber, String toFileName) {

        // Using the properties of the specified "to" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(toFileName);

        // Write the provided number to the newly declared filename.
        // See Note 3 above.
        ReadWriteFile.writeFile(myFileName, String.valueOf(myNumber));

//        telemetry.addData("Filename", toFileName);
//        telemetry.addData("Number being written", myNumber);
//        telemetry.update();         // display info on Driver Station screen

    }   // end of method writeToFile()

    /**
     * Writes a given string to a specified file on the Robot Controller.
     * If the file does not exist, it will be created in the FIRST/settings folder.
     *
     * @author Google Gemini
     *
     * @param dataToWrite The string data to be written to the file.
     * @param fileName The name of the file to write to (e.g., "mydata.txt").
     */
    public static void writeToTextFile(String dataToWrite, String fileName) {
        // Get the file object for the specified filename.
        // This will create the file in the FIRST/settings directory if it doesn't exist.
        File myFile = AppUtil.getInstance().getSettingsFile(fileName);

        // Write the data to the file.
        ReadWriteFile.writeFile(myFile, dataToWrite);
    }
}
