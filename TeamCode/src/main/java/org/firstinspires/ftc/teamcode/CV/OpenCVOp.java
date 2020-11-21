package org.firstinspires.ftc.teamcode.CV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotDrive;

@Autonomous(name="Test OpenCV", group = "Autonomous")
public class OpenCVOp extends LinearOpMode {
    OpenCVClass openCV = new OpenCVClass();
    RobotDrive robot = new RobotDrive();
    RingDeterminationPipeline ringDeterm = new RingDeterminationPipeline(RobotDrive.allianceColor.red);
    GoalDeterminationPipeline goalDeterm = new GoalDeterminationPipeline(RobotDrive.allianceColor.red);

    public void runOpMode(){

        openCV.initOpenCV(hardwareMap, telemetry, goalDeterm);

        waitForStart();
        openCV.startStream(openCV.PHONE_CAM);

        //openCV.togglePhoneFlash(true);
            while (opModeIsActive()) {
                /*telemetry.addData("Target Point: ", goalDeterm.getTargetPoint());
                telemetry.addData("Line Matrix: ", goalDeterm.getLineMatrix());
                telemetry.update();*/
            }
            openCV.stopStream();
            openCV.closeCamera();
    }
}
