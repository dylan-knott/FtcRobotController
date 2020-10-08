package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test OpenCV", group = "Autonomous")
public class OpenCVOp extends LinearOpMode {
    OpenCVClass openCV = new OpenCVClass();
    RingDeterminationPipeline ringDeterm = new RingDeterminationPipeline();

    public void runOpMode() {
        openCV.initOpenCV(hardwareMap, telemetry);

        waitForStart();
            openCV.startStream();


            while (opModeIsActive()) {
                telemetry.addLine("Ring Position: " + ringDeterm.getAnalysis());
                telemetry.update();
            }
    }
}
