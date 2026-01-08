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
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;


/** Ramp Camera Subsystem
 * @author kw126
 * @author Glenn
 * @author knutt5
 */
public class ArtifactCamera extends SubsystemBase {

    // Camera vision portal
    private volatile VisionPortal visionPortal;

    // Available vision processors to be used with camera
    // create a processor for each colour of field object
    // private volatile AprilTagProcessor aprilTagProcessor;
    private volatile ColorBlobLocatorProcessor TLpurpleBlobProcessor;
    private volatile ColorBlobLocatorProcessor TLgreenBlobProcessor;
    private volatile ColorBlobLocatorProcessor TRpurpleBlobProcessor;
    private volatile ColorBlobLocatorProcessor TRgreenBlobProcessor;
    private volatile ColorBlobLocatorProcessor BottompurpleBlobProcessor;
    private volatile ColorBlobLocatorProcessor BottomgreenBlobProcessor;


    // current selected vision mode
    private VisionProcessorMode currentMode = VisionProcessorMode.NONE;

    /**
     * Place code here to initialize subsystem
     */
    public ArtifactCamera(String cameraName) {

        TLpurpleBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                //.setTargetColorRange(new ColorRange(ColorSpace.RGB,
                //                    new Scalar(50.0, 0, 50.0),
                //                    new Scalar(255.0, 175.0, 255.0)))
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1.0, -0.4, -0.0))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setRoiColor(Color.rgb(255,255,255))
                .setBoxFitColor(Color.rgb(255, 0, 0))       // Disable the drawing of rectangles
                //.setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // available filtering functions
        TLpurpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 600, 76800));
        //TLpurpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1));
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 2.0));
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.8, 1.0));

        // tell blob processor how to sort its results
        TLpurpleBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));


        TLgreenBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
//                .setTargetColorRange(new ColorRange(ColorSpace.HSV,
//                                   new Scalar(0, 40, 0),
//                                   new Scalar(332, 27, 54)))
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1.0, -0.4, -0.0))// entire screen / screen size
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setRoiColor(Color.rgb(255,255,255))
                .setBoxFitColor(Color.rgb(0, 255, 0))       // Disable the drawing of rectangles
                //.setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // available filtering functions
        TLgreenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 600, 76800));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 2.0));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.8, 1.0));

        // tell blob processor how to sort its results
        TLgreenBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));


        TRpurpleBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
//                .setTargetColorRange(new ColorRange(ColorSpace.HSV,
//                        new Scalar(50.0, 0, 50.0),
//                        new Scalar(296.0, 59.0, 84.0)))
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.6, 1, 1.0, -0.0))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setRoiColor(Color.rgb(255,255,255))
                .setBoxFitColor(Color.rgb(255, 0, 0))       // Disable the drawing of rectangles
                //.setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // available filtering functions
        TRpurpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 200, 76800));
        //TLpurpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1));
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 2.0));
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.8, 1.0));

        // tell blob processor how to sort its results
        TRpurpleBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));


        TRgreenBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
//                .setTargetColorRange(new ColorRange(ColorSpace.HSV,
//                       new Scalar(0, 40, 0),
//                        new Scalar(143, 77, 62)))
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.6, 1, 1.0, -0.0))// entire screen / screen size
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setRoiColor(Color.rgb(255,255,255))
                .setBoxFitColor(Color.rgb(0, 255, 0))       // Disable the drawing of rectangles
                //.setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // available filtering functions
        TRgreenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 200, 76800));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 2.0));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.8, 1.0));

        // tell blob processor how to sort its results
        TRgreenBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));


// BOttom

        BottompurpleBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                //.setTargetColorRange(new ColorRange(ColorSpace.RGB,
                //        new Scalar(50.0, 0, 50.0),
                //        new Scalar(255.0, 175.0, 255.0)))
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, -0.6, 1.0, -0.8))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setRoiColor(Color.rgb(255,255,255))
                .setBoxFitColor(Color.rgb(255, 0, 0))       // Disable the drawing of rectangles
                //.setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // available filtering functions
        BottompurpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 700, 76800));
        //TLpurpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1));
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 2.0));
        //purpleBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.8, 1.0));

        // tell blob processor how to sort its results
        BottompurpleBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));


        BottomgreenBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
//                .setTargetColorRange(new ColorRange(ColorSpace.HSV,
//                        new Scalar(0, 40, 0),
//                        new Scalar(134, 23.0, 49)))
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, -0.6, 1.0, -0.8))// entire screen / screen size
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setRoiColor(0)
                .setBoxFitColor(Color.rgb(0, 255, 0))       // Disable the drawing of rectangles
                //.setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // available filtering functions
        BottomgreenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 700, 76800));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO, 1.0, 2.0));
        //greenBlobProcessor.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
        //        ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY, 0.8, 1.0));

        // tell blob processor how to sort its results
        BottomgreenBlobProcessor.setSort(new ColorBlobLocatorProcessor.BlobSort(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));


        // Build the camera vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(160, 120))
                //.setCameraResolution(new Size(1280,720)) if have an HD camera
                .addProcessors(TLpurpleBlobProcessor, TLgreenBlobProcessor, TRpurpleBlobProcessor, TRgreenBlobProcessor, BottompurpleBlobProcessor, BottomgreenBlobProcessor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // by default disable all processors
        setVisionProcessingMode(VisionProcessorMode.ARTIFACT);

        // show camera stream on panels dashboard
        enableDashBoardView(true);
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
    }

    /**
     * use this function to set vision processing mode of the camera
     *
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
                visionPortal.setProcessorEnabled(TLpurpleBlobProcessor, false);
                visionPortal.setProcessorEnabled(TLgreenBlobProcessor, false);
                visionPortal.setProcessorEnabled(TRpurpleBlobProcessor, false);
                visionPortal.setProcessorEnabled(TRgreenBlobProcessor, false);
                visionPortal.setProcessorEnabled(BottompurpleBlobProcessor, false);
                visionPortal.setProcessorEnabled(BottomgreenBlobProcessor, false);
                break;
            case ARTIFACT:
                currentMode = VisionProcessorMode.ARTIFACT;
                //SetAutoCameraExposure();
                //setCameraExposure(60, 350);
                visionPortal.setProcessorEnabled(TLpurpleBlobProcessor, true);
                visionPortal.setProcessorEnabled(TLgreenBlobProcessor, true);
                visionPortal.setProcessorEnabled(TRpurpleBlobProcessor, true);
                visionPortal.setProcessorEnabled(TRgreenBlobProcessor, true);
                visionPortal.setProcessorEnabled(BottompurpleBlobProcessor, true);
                visionPortal.setProcessorEnabled(BottomgreenBlobProcessor, true);
                //visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                break;
            default:
                // don't change anything
        }

        RobotContainer.ActiveOpMode.sleep(100);
        enableDashBoardView(true);
    }

    // use this function to set vision processing mode of the camera

    /**
     * used to get the current vision processing mode
     */
    public VisionProcessorMode getVisionProcessingMode() {
        return currentMode;
    }

    // ---------- Colour blob access functions ----------

    // get all blob detections (if any) from camera
    // returns list containing info on each blob
    /*public List<ColorBlobLocatorProcessor.Blob> GetAllBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch(currentMode){
            case ARTIFACT:
                blobs = TLpurpleBlobProcessor.getBlobs();
                blobs.addAll( TLgreenBlobProcessor.getBlobs());
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
        if (blobs!= null && !blobs.isEmpty()){
            blobs = Collections.singletonList(blobs.get(0));
        }
        return blobs;
    }

    // gets all purple blobs
    public List<ColorBlobLocatorProcessor.Blob> GetPurpleBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch(currentMode){
            case ARTIFACT:
                blobs = TLpurpleBlobProcessor.getBlobs();
                break;
            default:
                blobs = new ArrayList<>();
        }

        // available sorting functions
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        // use if we only want to return the top item (if any) and disregard rest
        if (blobs!= null && !blobs.isEmpty()){
            blobs = Collections.singletonList(blobs.get(0));
        }

        return blobs;
    }

    // gets all green blobs
    public List<ColorBlobLocatorProcessor.Blob> GetGreenBlobDetections() {
        List<ColorBlobLocatorProcessor.Blob> blobs;

        switch(currentMode){
            case ARTIFACT:
                blobs =  TLgreenBlobProcessor.getBlobs();
                break;
            default:
                blobs = new ArrayList<>();
        }

        // available sorting functions
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        // use if we only want to return the top item (if any) and disregard rest
        if (blobs!= null && !blobs.isEmpty()){
            blobs = Collections.singletonList(blobs.get(0));
        }

        return blobs;
    }*/


    private Point calculateCenter(Rect boundingBox) {

        double centerX = boundingBox.x + boundingBox.width / 2.0;
        double centerY = boundingBox.y + boundingBox.height / 2.0;
        return new Point(centerX, centerY);
    }


    // ---------- General Camera access functions

    // enables camera view in dashboard
    public void enableDashBoardView(boolean enable) {
        if (enable)
            FtcDashboard.getInstance().startCameraStream(visionPortal, 4);
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
    public double GetCameraFPS() {
        return visionPortal.getFps();
    }

    public boolean cameraReady() {
        return (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    /**
     * Set the camera gain and exposure.
     */
    private void setCameraExposure(int exposureMS, int gain) {

        // wait until camera in streaming mode
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }

        // set exposure control to manual
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);

            RobotContainer.ActiveOpMode.sleep(50);
        }

        // set exposure and gain
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        RobotContainer.ActiveOpMode.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        RobotContainer.ActiveOpMode.sleep(20);
    }

    /**
     * Sets the camera exposure to automatic
     */
    private void SetAutoCameraExposure() {

        // wait until camera in streaming mode
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }

        // set camera to auto
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Auto);
    }

    public enum ArtifactColours{
        Purple,
        Green,
        Nothing
    }
    public ArtifactColours getLeftColour(){
        List<ColorBlobLocatorProcessor.Blob> blobs;
       blobs =  TLgreenBlobProcessor.getBlobs();
        if(blobs != null && !blobs.isEmpty()){
            return ArtifactColours.Green;
        }else{
            blobs =  TLpurpleBlobProcessor.getBlobs();
            if(blobs != null && !blobs.isEmpty()){
                return ArtifactColours.Purple;
            }else{
                return ArtifactColours.Nothing;
            }
        }
    }
    public ArtifactColours getRightColour(){
        List<ColorBlobLocatorProcessor.Blob> blobs;
        blobs =  TRgreenBlobProcessor.getBlobs();
        if(blobs != null && !blobs.isEmpty()){
            return ArtifactColours.Green;
        }else{
            blobs =  TRpurpleBlobProcessor.getBlobs();
            if(blobs != null && !blobs.isEmpty()){
                return ArtifactColours.Purple;
            }else{
                return ArtifactColours.Nothing;
            }
        }
    }
    public ArtifactColours getBottomColour(){
        List<ColorBlobLocatorProcessor.Blob> blobs;
        blobs =  BottomgreenBlobProcessor.getBlobs();
        if(blobs != null && !blobs.isEmpty()){
            return ArtifactColours.Green;
        }else{
            blobs =  BottompurpleBlobProcessor.getBlobs();
            if(blobs != null && !blobs.isEmpty()){
                return ArtifactColours.Purple;
            }else{
                return ArtifactColours.Nothing;
            }
        }
    }

    public boolean IsLeftPresent(){
        if(getLeftColour() != ArtifactColours.Nothing){
            return true;
        }else{
            return false;
        }
    }

    public boolean IsRightPresent(){
        if(getRightColour() != ArtifactColours.Nothing){
            return true;
        }else{
            return false;
        }
    }

    public boolean IsBottomPresent(){
        if(getBottomColour() != ArtifactColours.Nothing){
            return true;
        }else{
            return false;
        }
    }
}