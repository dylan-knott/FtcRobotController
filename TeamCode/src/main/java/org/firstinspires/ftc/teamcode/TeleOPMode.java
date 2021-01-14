package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;

@TeleOp(name= "TeleOp", group= "TeleOp")
public class TeleOPMode extends LinearOpMode {
    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    APMecanumDrive drive = null;

    public void runOpMode() throws InterruptedException{
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        drive = robot.rrDrive;

        waitForStart();

        while (opModeIsActive()) {

            //This code is not currently active as we are transitioning from old tele-op to a road-runner implemented tele-op
            /*
            if (gamepad1.left_bumper) robot.motorPower = 0.2;
            else if (gamepad1.right_bumper) robot.motorPower= 0.15;
            else robot.motorPower = 0.8;
            */

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            //Gamepad 1  ***Drivetrain***
            if (gamepad1.x) robot.releaseIntake();
            if (gamepad1.y) robot.toggleRamp();

            //Gamepad 2  ***Gun and intake***
            robot.enableIntake(gamepad2.right_stick_y);
            robot.setFlywheelsRPM(gamepad2.right_trigger);

        }
        }

    }
