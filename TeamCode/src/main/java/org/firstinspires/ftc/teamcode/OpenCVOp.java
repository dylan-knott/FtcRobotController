package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(name="Test OpenCV", group = "Autonomous")
public class OpenCVOp extends LinearOpMode {
    OpenCVClass openCV = new OpenCVClass();
    RobotDrive robot = new RobotDrive();
    RingDeterminationPipeline ringDeterm = new RingDeterminationPipeline(RobotDrive.allianceColor.red);
    GoalDeterminationPipeline goalDeterm = new GoalDeterminationPipeline(RobotDrive.allianceColor.red);

    public void runOpMode() throws InterruptedException{

        openCV.initOpenCV(hardwareMap, telemetry, goalDeterm);
        openCV.startStream();
        waitForStart();

        //openCV.togglePhoneFlash(true);
            while (opModeIsActive()) {
                telemetry.addData("Target Point: ", goalDeterm.getTargetPoint());
                telemetry.addData("Line Matrix: ", goalDeterm.getLineMatrix());
                telemetry.update();
            }
            openCV.togglePhoneFlash(false);
            openCV.stopStream();
            openCV.closeCamera();
    }
}
