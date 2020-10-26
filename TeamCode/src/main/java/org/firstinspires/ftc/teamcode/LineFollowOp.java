package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Line Follower", group = "Autonomous")
public class LineFollowOp extends LinearOpMode {
    OpenCVClass openCV = new OpenCVClass();
    RobotDrive robot = new RobotDrive();
    LineFollowingPipeline lf = new LineFollowingPipeline(RobotDrive.allianceColor.red);

    final int LINE_THRESH = 150;
    double motorPower;
    double motorCorrect;

    int[] lineVals;
    int left;
    int middle;
    int right;

    public void runOpMode() throws InterruptedException {
        //initialization code
        openCV.initOpenCV(hardwareMap, telemetry, lf);
        openCV.startStream();

        waitForStart();
        //Runs after start button is pressed
        openCV.togglePhoneFlash(true);

        motorPower = 0.5;
        //Positive motor correct = turn right
        motorCorrect = 0;


        while (opModeIsActive()) {
            lineVals = lf.getRegionAnalysis(); //Returns Left, Middle, then Right brightness value
            left = lineVals[0];
            middle = lineVals[1];
            right = lineVals[2];

            //lower value = line

            if (left > middle && left > right) {
                //line is on the right
                motorCorrect = 0.2;
                setMotors();
            } else if (right > middle && right > left) {
                //line is on the left
                motorCorrect = -0.2;
                setMotors();
            } else if (middle < right && middle < left) {
                //line is centered
                motorCorrect = 0;
                setMotors();
            }
            else if (right >= LINE_THRESH && middle >= LINE_THRESH && left >= LINE_THRESH) {
                motorPower = 0;
                motorCorrect = 0;
                setMotors();
                wait(1000);
                motorPower = 0.5;
                setMotors();
            } else {
                //End of the line
                motorPower = 0;
                motorCorrect = 0;
                setMotors();
            }

        }
    }
    void setMotors() {
        robot.leftFront.setPower(motorPower + motorCorrect);
        robot.leftRear.setPower(motorPower + motorCorrect);
        robot.rightFront.setPower(motorPower - motorCorrect);
        robot.rightRear.setPower(motorPower - motorCorrect);
    }


}
