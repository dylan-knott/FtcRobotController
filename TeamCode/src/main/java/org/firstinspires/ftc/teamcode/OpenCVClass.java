package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.disnodeteam.dogecv.*;


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

    //Some constants regarding sizing of elements
    static final int SCREEN_HEIGHT = 240;
    static final int SCREEN_WIDTH = 320;
    static final int REGION_HEIGHT = 120; //Sensing region will take up middle 50% of the camera's height.
    static final int REGION_COUNT = 8;
    static final int REGION_Y_ANCHOR = 60; //Marks the top of the sensing regions
    int regionWidth;

    //Image mats for processing
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat Cr = new Mat();
    Mat[] CbRegions = new Mat[REGION_COUNT];
    Mat[] CrRegions = new Mat[REGION_COUNT];
    int[] regionAnalysis;
    Scalar GREEN = new Scalar(0, 255, 0);

    //Point p1;
    //Point p2;


    public void init(Mat firstFrame) {
        int regionLeft;
        regionWidth = SCREEN_WIDTH / REGION_COUNT;
        convImage(firstFrame);

        //Split original image into submats for processing
        for (int i = 0; i < REGION_COUNT; i++) {
            regionLeft = i * regionWidth;
            CbRegions[i] = Cb.submat(regionLeft, regionLeft + regionWidth, REGION_Y_ANCHOR, REGION_Y_ANCHOR + REGION_HEIGHT);
        }
        //Draw rectangle on the screen
        for (int i = 0; i < SCREEN_WIDTH; i += regionWidth) {
            Point p1 = new Point(i * regionWidth, REGION_Y_ANCHOR);
            Point p2 = new Point(i * regionWidth + regionWidth, REGION_Y_ANCHOR + REGION_HEIGHT);
            Imgproc.rectangle(firstFrame, p1, p2, GREEN, 2);
        }
    }

    void convImage(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    @Override
    public Mat processFrame(Mat input) {
        convImage(input);

        //Draw rectangle on the screen
        for (int i = 0; i < SCREEN_WIDTH; i += regionWidth) {
            Point p1 = new Point(i, REGION_Y_ANCHOR);
            Point p2 = new Point(p1.x + regionWidth, REGION_Y_ANCHOR + REGION_HEIGHT);
            Imgproc.rectangle(input, new Rect(p1, p2), GREEN, 2);
        }
        return input;
    }

    void analyzeFrame() {
        int avg;
        if (teamColor == RobotDrive.allianceColor.blue) {
            for (int i = 0; i < REGION_COUNT; i ++) { // Loop through regions averaging values and populating the array
                avg = (int) Core.mean(CbRegions[i]).val[0];
                regionAnalysis[i] = avg;
            }
        }
        else if (teamColor == RobotDrive.allianceColor.red) {
            for (int i = 0; i < REGION_COUNT; i ++) { // Loop through regions averaging values and populating the array
                avg = (int) Core.mean(CrRegions[i]).val[0];
                regionAnalysis[i] = avg;
            }
        }


    }

    public int[] getRegionAnalysis() { return regionAnalysis; }
}