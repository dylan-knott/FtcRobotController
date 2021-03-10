package org.firstinspires.ftc.teamcode.drive.opmode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CV.TensorFlowRingIdentification;

@Autonomous(name="TensorFlow Test")
public class TFTest extends LinearOpMode {
    TensorFlowRingIdentification tf = new TensorFlowRingIdentification();
    public void runOpMode() {

        tf.initObjectDetector(hardwareMap, telemetry);
        waitForStart();

        while (!isStopRequested()){
            tf.runDetect(30);
        }

    }
}
