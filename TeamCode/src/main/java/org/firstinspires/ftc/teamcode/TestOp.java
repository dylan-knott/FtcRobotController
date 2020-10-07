package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestOp extends LinearOpMode {
    RobotDrive robot = new RobotDrive();
    TestClass localInstance1 = new TestClass();

    public void runOpMode() {
        //Init button is hit
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);


        //Run one time when start button is pressed
        waitForStart();



        //Run repeatedly once op mode is started
        while(opModeIsActive()) {

        }
    }
}
