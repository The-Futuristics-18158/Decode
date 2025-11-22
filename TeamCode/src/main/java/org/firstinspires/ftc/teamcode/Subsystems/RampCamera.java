package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;
import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.SortOrder;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utility.VisionProcessorMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


/** Ramp Camera Subsystem
 * @author kw126
 * @author Glenn
 * @author knutt5
 */
public class RampCamera extends SubsystemBase {

    // Camera vision portal
    private volatile VisionPortal visionPortal;

    // Available vision processors to be used with camera
    // create a processor for each colour of field object
    // private volatile AprilTagProcessor aprilTagProcessor;
    private volatile ColorBlobLocatorProcessor purpleBlobProcessor;
    private volatile ColorBlobLocatorProcessor greenBlobProcessor;

    // current selected vision mode
    private VisionProcessorMode currentMode = VisionProcessorMode.NONE;

    /** Place code here to initialize subsystem */
    public RampCamera(String cameraName) {

        // Build the AprilTag processor
        //aprilTagProcessor = new AprilTagProcessor.Builder()
        //        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        //        .setDrawTagID(true)
        //        .setDrawTagOutline(true)
        //        .setDrawAxes(true)
        //        .setDrawCubeProjection(true)
        //        .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
        //        //.setNumThreads(tbd)
        //        .build();
        // set apriltag resolution decimation factor
        //SetDecimation(2);

        purpleBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                //.setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                //                    new Scalar(32.0, 135.0, 135.0),
                //                    new Scalar(255.0, 155.0, 169.0)))
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // available filtering functions
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 6000, 40000));
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 2.0));
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.8, 1.0));

        // tell blob processor how to sort its results
        purpleBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));


        greenBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                //.setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                //                    new Scalar(32.0, 50.0, 118.0),
                //                    new Scalar(255.0, 105.0, 145.0)))
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // available filtering functions
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 6000, 40000));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 2.0));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.8, 1.0));

        // tell blob processor how to sort its results
        greenBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));

        // Build the camera vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(320, 240))
                //.setCameraResolution(new Size(1280,720)) if have an HD camera
                .addProcessors(purpleBlobProcessor, greenBlobProcessor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // by default disable all processors
        setVisionProcessingMode(VisionProcessorMode.ARTIFACT);

        // show camera stream on panels dashboard
        enableDashBoardView(true);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
    }

    /**use this function to set vision processing mode of the camera
     * @param newMode the new mode from VisionProcessorMode Enum
     */
    public void setVisionProcessingMode(@NonNull VisionProcessorMode newMode) {

        enableDashBoardView(false);
        RobotContainer.ActiveOpMode.sleep(100);

        // enable/disable modes depending on vision processing mode selected
        //visionPortal.stopStreaming();
        switch (newMode) {
            case NONE:
                currentMode = VisionProcessorMode.NONE;
                //SetAutoCameraExposure();
                //setCameraExposure(60, 350);
                visionPortal.setProcessorEnabled(purpleBlobProcessor, false);
                visionPortal.setProcessorEnabled(greenBlobProcessor, false);
                //visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                break;
            case ARTIFACT:
                currentMode = VisionProcessorMode.ARTIFACT;
                //SetAutoCameraExposure();
                //setCameraExposure(60, 350);
                visionPortal.setProcessorEnabled(purpleBlobProcessor, true);
                visionPortal.setProcessorEnabled(greenBlobProcessor, true);
                //visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                break;
            default:
                // don't change anything
        }

        RobotContainer.ActiveOpMode.sleep(100);
        enableDashBoardView(true);
    }

    // use this function to set vision processing mode of the camera
    /** used to get the current vision processing mode
     */
    public VisionProcessorMode getVisionProcessingMode()
    { return currentMode; }



    // ---------- Apriltag Access functions ----------

    // get fresh AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    /*public List<AprilTagDetection> GetFreshAprilTagDetections() {
        if (currentMode==VisionProcessorMode.APRIL_TAG_ONLY)
            return aprilTagProcessor.getFreshDetections();
        else
            return new ArrayList<>();
    } */

    // get current AprilTag detections (if any) from camera
    // returns list containing info on each tag detected
    /*public List<AprilTagDetection> GetCurrentAprilTagDetections() {
        if (currentMode==VisionProcessorMode.APRIL_TAG_ONLY)
            return aprilTagProcessor.getDetections();
        else
            return new ArrayList<>();
    } */

    // sets decimation of AprilTag processing
    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // eg: Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    /*public void SetDecimation (int num) {
        aprilTagProcessor.setDecimation(num);
    } */


    // ---------- Colour blob access functions ----------

    // get all blob detections (if any) from camera
    // returns list containing info on each blob
    public List<ColorBlobLocatorProcessor.Blob> GetAllBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch(currentMode){
            case ARTIFACT:
                blobs = purpleBlobProcessor.getBlobs();
                blobs.addAll(greenBlobProcessor.getBlobs());
                break;
            default:
                blobs = new ArrayList<>();
        }

        // available filtering functions
        //ColorBlobLocatorProcessor.Util.filterByArea(25, 300000, blobs);
        //ColorBlobLocatorProcessor.Util.filterByDensity(0.10, 1.0, blobs);
        //ColorBlobLocatorProcessor.Util.filterByAspectRatio(0.1, 10.0, blobs);

        // available sorting functions
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
        //ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, SortOrder.DESCENDING, blobs);
        //ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, SortOrder.DESCENDING, blobs);
        //ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, SortOrder.DESCENDING, blobs);

        // use if we only want to return the top item (if any) and disregard rest
        //blobs = Collections.singletonList(blobs.get(0));

        return blobs;
    }

    // gets all purple blobs
    public List<ColorBlobLocatorProcessor.Blob> GetPurpleBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch(currentMode){
            case ARTIFACT:
                blobs = purpleBlobProcessor.getBlobs();
                break;
            default:
                blobs = new ArrayList<>();
        }

        // available sorting functions
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        // use if we only want to return the top item (if any) and disregard rest
        //blobs = Collections.singletonList(blobs.get(0));

        return blobs;
    }

    // gets all green blobs
    public List<ColorBlobLocatorProcessor.Blob> GetGreenBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch(currentMode){
            case ARTIFACT:
                blobs = greenBlobProcessor.getBlobs();
                break;
            default:
                blobs = new ArrayList<>();
        }

        // available sorting functions
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        // use if we only want to return the top item (if any) and disregard rest
        //blobs = Collections.singletonList(blobs.get(0));

        return blobs;
    }


    private Point calculateCenter(Rect boundingBox) {

        double centerX = boundingBox.x + boundingBox.width / 2.0;
        double centerY = boundingBox.y + boundingBox.height / 2.0;
        return new Point(centerX, centerY);
    }


    // ---------- General Camera access functions

    // enables camera view in dashboard
    public void enableDashBoardView(boolean enable) {
        if (enable)
            FtcDashboard.getInstance().startCameraStream(visionPortal,4);
        else
            FtcDashboard.getInstance().stopCameraStream();
    }

    public void enableLiveView(boolean enable) {
        if (enable)
            visionPortal.resumeLiveView();
        else
            visionPortal.stopLiveView();
    }

    // get camera frames per second
    public double GetCameraFPS () {
        return visionPortal.getFps();
    }

    public boolean cameraReady(){
        return( visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    /** Set the camera gain and exposure. */
    private void setCameraExposure(int exposureMS, int gain) {

        // wait until camera in streaming mode
        while (visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING)
        {}

        // set exposure control to manual
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);

            RobotContainer.ActiveOpMode.sleep(50);
        }

        // set exposure and gain
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        RobotContainer.ActiveOpMode.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        RobotContainer.ActiveOpMode.sleep(20);
    }

    /** Sets the camera exposure to automatic */
    private void SetAutoCameraExposure() {

        // wait until camera in streaming mode
        while (visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING)
        {}

        // set camera to auto
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Auto);
    }


}
