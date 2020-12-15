package org.firstinspires.ftc.teamcode.CV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;

@Autonomous(name="Test OpenCV", group = "Autonomous")
public class OpenCVOp extends LinearOpMode {
    OpenCVClass openCV = new OpenCVClass();
    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    RingDeterminationPipeline ringDeterm = new RingDeterminationPipeline(LocalizedRobotDrive.allianceColor.red);
    GoalDeterminationPipeline goalDeterm = new GoalDeterminationPipeline(LocalizedRobotDrive.allianceColor.red);
    IntakePipeline intakePipeline = new IntakePipeline();
    SamplePipeline samplePipeline = new SamplePipeline();

    public void runOpMode(){

        openCV.initOpenCV(hardwareMap, telemetry, ringDeterm, intakePipeline);

        waitForStart();
        //openCV.startStream(openCV.PHONE_CAM);

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
