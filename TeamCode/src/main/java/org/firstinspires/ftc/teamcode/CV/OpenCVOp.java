package org.firstinspires.ftc.teamcode.CV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;

@Autonomous(name="Test OpenCV", group = "Autonomous")
@Disabled
public class OpenCVOp extends LinearOpMode {
    OpenCVClass openCV = new OpenCVClass();
    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    RingDeterminationPipeline ringDeterm = new RingDeterminationPipeline(LocalizedRobotDrive.allianceColor.red);
    GoalDeterminationPipeline goalDeterm = new GoalDeterminationPipeline(LocalizedRobotDrive.allianceColor.red);
    IntakePipeline intakePipeline = new IntakePipeline();

    public void runOpMode(){

        openCV.initOpenCV(hardwareMap, telemetry, goalDeterm, intakePipeline);

        openCV.startStream();
        waitForStart();
        openCV.togglePhoneFlash(true);
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
