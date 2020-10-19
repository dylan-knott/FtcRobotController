package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.LineSegmentDetector;
import org.openftc.easyopencv.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.disnodeteam.dogecv.*;

import java.util.*;


public class OpenCVClass {

    public OpenCvInternalCamera phoneCam;



    public void initOpenCV(HardwareMap hardwareMap, Telemetry telem, OpenCvPipeline pipeline) {
        //Initialization code

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setPipeline(pipeline);
        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

    }

    public void startStream() {
        // Start streaming
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


                //Turn on the phone's flash
                phoneCam.setFlashlightEnabled(true);
            }
        });

    }

    public void stopStream() {
        phoneCam.stopStreaming();
            }

    public void closeCamera() {
        phoneCam.closeCameraDevice();
    }
}

class SamplePipeline extends OpenCvPipeline
{

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return input;
    }

}

class RingDeterminationPipeline extends OpenCvPipeline
{
    public RingDeterminationPipeline(RobotDrive.allianceColor allianceColor) {
        teamColor = allianceColor;
    }

    /*
     * An enum to define the number of rings
     */

    RobotDrive.allianceColor teamColor;
    public enum RingPosition
    {
        FOUR,
        ONE,
        NONE
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

    static final int REGION_WIDTH = 35;
    static final int REGION_HEIGHT = 25;

    final int FOUR_RING_THRESHOLD = 150;
    final int ONE_RING_THRESHOLD = 135;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1;

    // Volatile since accessed by OpMode thread w/o synchronization
    public volatile RingPosition position = RingPosition.FOUR;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        position = RingPosition.FOUR; // Record our analysis
        if(avg1 > FOUR_RING_THRESHOLD){
            position = RingPosition.FOUR;
        }else if (avg1 > ONE_RING_THRESHOLD){
            position = RingPosition.ONE;
        }else{
            position = RingPosition.NONE;
        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                -1); // Negative thickness means solid fill

        return input;
    }

    public int getAnalysis() { return avg1; }
    public RingPosition getPosition() { return position; }
}

class GoalDeterminationPipeline extends OpenCvPipeline {
    //Screen size 320 by 240

    //Constructor
    public GoalDeterminationPipeline(RobotDrive.allianceColor allianceColor) {
        int regionLeft;
        teamColor = allianceColor;
   }

    RobotDrive.allianceColor teamColor;

    //Some constants
    static final int SCREEN_HEIGHT = 240;
    static final int SCREEN_WIDTH = 320;

    //Constants regarding sizing of elements for region detection
    static final int REGION_HEIGHT = 120; //Sensing region will take up middle 50% of the camera's height.
    static final int REGION_COUNT = 8;
    static final int REGION_Y_ANCHOR = 60; //Marks the top of the sensing regions
    int regionWidth;

    //Constants for line-segment detection
    static final int INTENSITY_THRESHOLD = 150;

    //Image mats for processing
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();
    private Mat Cr = new Mat();
    private Mat activeMat = new Mat();


    //Used for Array-style detection of Target
    Mat[] CbRegions = new Mat[REGION_COUNT];
    Mat[] CrRegions = new Mat[REGION_COUNT];
    Mat[] activeRegions = new Mat[REGION_COUNT];
    int[]regionAvg;

    //Used for line-segment style detection of target
    private Mat thresholdMap;
    //private LineSegmentDetector lsd = Imgproc.createLineSegmentDetector();
    private MatOfFloat4 lines = new MatOfFloat4();
    private Mat avgs;
    double[] vector;
    private int xAvg;
    private int yAvg;

    Scalar GREEN = new Scalar(0, 255, 0);


    public void init(Mat firstFrame) {
        int regionLeft;
        regionWidth = SCREEN_WIDTH / REGION_COUNT;
        convImage(firstFrame);

        //Split original image into submats for processing
        for (int i = 0; i < REGION_COUNT; i++) {
            regionLeft = i * regionWidth;
            CbRegions[i] = Cb.submat(regionLeft, regionLeft + regionWidth, REGION_Y_ANCHOR, REGION_Y_ANCHOR + REGION_HEIGHT);
            CrRegions[i] = Cr.submat(regionLeft, regionLeft + regionWidth, REGION_Y_ANCHOR, REGION_Y_ANCHOR + REGION_HEIGHT);
        }
    }

    // Convert image into the YCrCb color space, and extract channels into their own matrices
    void convImage(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    //Runs every frame
    @Override
    public Mat processFrame(Mat input) {
        convImage(input);

        if (teamColor == RobotDrive.allianceColor.blue) {
            activeMat = Cb;
            activeRegions = CbRegions;
        }
        else{
            activeMat = Cr;
            activeRegions = CrRegions;
        }


        //Draw rectangle for region on the screen
        for (int i = 0; i < SCREEN_WIDTH; i += regionWidth) { //Counts through the regions, counting by region widths in terms of pixels
            Point p1 = new Point(i, REGION_Y_ANCHOR);
            Point p2 = new Point(p1.x + regionWidth, REGION_Y_ANCHOR + REGION_HEIGHT);
            Imgproc.rectangle(input, new Rect(p1, p2), GREEN, 2);
        }

        Point midTarget = lineDetectFrame();

        Imgproc.circle(input, midTarget, 0, new Scalar(0, 0, 255), -1);

        //lsd.drawSegments(input, lines);
        return input;
    }

    Point lineDetectFrame() {
        Point targetPoint;
        //Apply a binary threshold to the active matrix to differentiate colors better
        Imgproc.threshold(activeMat, thresholdMap, INTENSITY_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        //Detect lines and return a matrix with X1 Y1 X2 Y2 for each line
        //lsd.detect(thresholdMap, lines);
        Imgproc.HoughLinesP(thresholdMap,lines, 1,Math.PI/180, 150);

        if (lines.empty()) { //No lines detected
            //TODO: No lines detected
        targetPoint = null;
        }
        else {
            //Average each column of the matrix and store it into a 1 dimensional matrix
            Core.reduce(lines, avgs, 0, Core.REDUCE_AVG);

            //Turn 1D Matrix into an array of doubles
            vector = avgs.get(0, 0);

            //Data will always stored as [X1, Y1, X2, Y2] so indices can be hard coded in
            xAvg = (int) ((vector[0] + vector[2]) / 2);
            yAvg = (int) ((vector[1] + vector[3]) / 2);//Find midpoints of all lines detected, this will be the center of the target

            targetPoint = new Point(xAvg, yAvg);
        }
        

        targetPoint = new Point(50, 50);
        return targetPoint;
    }

    void analyzeFrameArr() {
        int avg;
        for (int i = 0; i < REGION_COUNT; i ++) { // Loop through regions calculating average brightness values and populating the array with the averages
            avg = (int) Core.mean(activeRegions[i]).val[0];
            regionAvg[i] = avg;
        }
    }

    public int[] getRegionAnalysis() { return regionAvg; }
}