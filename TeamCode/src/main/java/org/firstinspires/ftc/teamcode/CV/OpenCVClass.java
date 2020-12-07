package org.firstinspires.ftc.teamcode.CV;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;
import org.firstinspires.ftc.teamcode.MultipleCameraExample;
import org.firstinspires.ftc.teamcode.RobotDrive;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;


public class OpenCVClass {

    //Values to determine if the active camera will be the webcam or phone camera
    public final int PHONE_CAM = 0;
    public final int WEB_CAM = 1;

    public OpenCvCamera activeCam;

    private OpenCvCamera phoneCam;
    private OpenCvCamera webCam;

    private Telemetry telemetry;

    public void initOpenCV(HardwareMap hardwareMap, Telemetry telem, OpenCvPipeline phonePipeline, OpenCvPipeline webcamPipeline) {
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webCam"), viewportContainerIds[1]);



        phoneCam.openCameraDevice();
        webCam.openCameraDevice();


        phoneCam.setPipeline(phonePipeline);
        webCam.setPipeline(webcamPipeline);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //startStream(PHONE_CAM);
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /**
         * This is the only thing you need to do differently when using multiple cameras.
         * Instead of obtaining the camera monitor view and directly passing that to the
         * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
         * on that view in order to split that view into multiple equal-sized child views,
         * and then pass those child views to the constructor.
         */
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webCam"), viewportContainerIds[1]);

        phoneCam.openCameraDevice();
        webCam.openCameraDevice();

        phoneCam.setPipeline(phonePipeline);
        webCam.setPipeline(webcamPipeline);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }



    public void startStream(int active) {

        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

       /* webCam.openCameraDevice(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                activeCam = webCam;
            }
        });
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                activeCam = webCam;
            }
        });

        switch (active) {
            case 0:
                telemetry.addLine("In switch" + active);
                telemetry.update();
                // Start streaming
                phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        telemetry.addLine("In onOpened");
                        telemetry.update();
                        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        activeCam = phoneCam;
                    }
                });
            break;
            case 1:
                // Start streaming
                webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        activeCam = webCam;
                    }
                });
            break;
        }*/
    }

    public void togglePhoneFlash(boolean state) {
        if (activeCam == phoneCam) {
            //phoneCam.setFlashlightEnabled(state);
        }
    }

    public void stopStream() {
        //activeCam.stopStreaming();
        phoneCam.stopStreaming();
        webCam.stopStreaming();
    }

    public void closeCamera() {
        //activeCam.closeCameraDevice();
        phoneCam.closeCameraDevice();
        webCam.closeCameraDevice();
    }
}

class SamplePipeline extends OpenCvPipeline {

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

class RingDeterminationPipeline extends OpenCvPipeline {
    public RingDeterminationPipeline(LocalizedRobotDrive.allianceColor allianceColor) {
        teamColor = allianceColor;
    }

    /*
     * An enum to define the number of rings
     */

    LocalizedRobotDrive.allianceColor teamColor;
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
    public GoalDeterminationPipeline(LocalizedRobotDrive.allianceColor allianceColor) {
        int regionLeft;
        teamColor = allianceColor;
   }

    LocalizedRobotDrive.allianceColor teamColor;

    //Some constants
    static final int SCREEN_HEIGHT = 320;
    static final int SCREEN_WIDTH = 240;


    //Constants for line-segment detection
    static final int INTENSITY_THRESHOLD = 150;

    //Image mats for processing
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();
    private Mat Cr = new Mat();
    private Mat ringMask = new Mat();
    private Mat activeMat = new Mat();


    //Used for line-segment style detection of target
    private final double DETECT_AREA = 0.33; //Ratio of the area of the screen that should be scanned for detecting the goal
    private Mat thresholdMap = new Mat();
    private Mat blurred = new Mat();
    private Mat resized = new Mat();
    private Mat edges = new Mat();
    private Mat lines = new Mat();
    private Mat X1Mat = new Mat();
    private Mat X2Mat = new Mat();
    private Mat Y1Mat = new Mat();
    private Mat Y2Mat = new Mat();
    private double[] X1Avgs;
    private double[] Y1Avgs;
    private double[] X2Avgs;
    private double[] Y2Avgs;
    private Point targetPoint = new Point(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    private double xAvg;
    private double yAvg;

    Scalar GREEN = new Scalar(0, 255, 0);


    public void init(Mat firstFrame) {
        convImage(firstFrame);
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
        lineDetectFrame();
        Imgproc.circle(input, targetPoint, 2, new Scalar(255+4, 0, 0), -1);
        return input;
    }

    void lineDetectFrame() {
        //Apply a binary threshold to the active matrix to differentiate colors better
        Imgproc.threshold(activeMat, thresholdMap, INTENSITY_THRESHOLD, 255, Imgproc.THRESH_BINARY);

        //Apply a blur to the image
        Imgproc.GaussianBlur(thresholdMap, blurred, new Size(3, 3), 1);

        //Detects edges in an image and only highlights those edges
        Imgproc.Canny(blurred, edges, 100, 200);

        //Cut off bottom portion of the image
        resized = blurred.submat(0, (int)(SCREEN_HEIGHT * DETECT_AREA), 0, SCREEN_WIDTH);
        //Returns a matrix with X1 Y1 X2 Y2 for each line detected
        Imgproc.HoughLinesP(resized, lines, 0.5,Math.PI/90, 10, 30, 10);

        if (!lines.empty()) { //Line detected
            //Extract X1, X2, Y1, and Y2 from the lines matrix into seperate mats
            Core.extractChannel(lines, X1Mat, 0);
            Core.extractChannel(lines, Y1Mat, 1);
            Core.extractChannel(lines, X2Mat, 2);
            Core.extractChannel(lines, Y2Mat, 3);

            X1Avgs = Core.mean(X1Mat).val;
            Y1Avgs = Core.mean(Y1Mat).val;
            X2Avgs = Core.mean(X2Mat).val;
            Y2Avgs = Core.mean(Y2Mat).val;

            xAvg = (X1Avgs[0] + X2Avgs[0]) / 2;
            yAvg = (Y1Avgs[0] + Y2Avgs[0]) / 2;
            //Average each column of the matrix and store it into a 1 dimensional matrix


            targetPoint = new Point (xAvg, yAvg);
        }
        else {
            //TODO: No lines detected
            targetPoint = new Point(0, 0);
        }
    }

    public String getTargetPoint() { return targetPoint.toString(); }

    public String getLineMatrix() {return lines.toString(); }
}

class IntakePipeline extends OpenCvPipeline {

    //Mats for processing
    Mat gray = new Mat();

    private void convertFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
    }

    public void init(Mat firstFrame) {
        convertFrame(firstFrame);

    }

    public Mat processFrame(Mat input)
    {
        convertFrame(input);

        return input;
    }

}