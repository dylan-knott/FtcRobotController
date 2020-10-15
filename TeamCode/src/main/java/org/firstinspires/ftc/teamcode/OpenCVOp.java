package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(name="Test OpenCV", group = "Autonomous")
public class OpenCVOp extends LinearOpMode {
    OpenCVClass openCV = new OpenCVClass();
    RobotDrive robot = new RobotDrive();
    RingDeterminationPipeline ringDeterm = new RingDeterminationPipeline(RobotDrive.allianceColor.blue);

    public void runOpMode() throws InterruptedException{

        openCV.initOpenCV(hardwareMap, telemetry, ringDeterm);

        waitForStart();
            openCV.startStream();


            while (opModeIsActive()) {
                telemetry.addLine("Yellow Value: " + ringDeterm.getAnalysis());
                telemetry.addLine("Ring Position: " + ringDeterm.getPosition());
                telemetry.update();
            }
    }
}
