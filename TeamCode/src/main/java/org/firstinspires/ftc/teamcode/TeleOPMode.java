package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name= "TeleOp", group= "TeleOp")
public class TeleOPMode extends LinearOpMode {
    LocalizedRobotDrive robot = new LocalizedRobotDrive();

    public void runOpMode() throws InterruptedException{
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);


        waitForStart();

        while (opModeIsActive()) {

            //This code is not currently active as we are transitioning from old tele-op to a road-runner implemented tele-op
            /*
            if (gamepad1.left_bumper) robot.motorPower = 0.2;
            else if (gamepad1.right_bumper) robot.motorPower= 0.15;
            else robot.motorPower = 0.8;
            */

            //Roadrunner-based movement code for teleop
            /*robot.rrDrive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );*/




            robot.rrDrive.update();

            Pose2d poseEstimate = robot.rrDrive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Right trigger values ", gamepad2.right_trigger);
            telemetry.update();

            //Gamepad 2  ***Gun and intake***
            robot.enableIntake(gamepad2.right_stick_y);
            //robot.setFlywheels(-gamepad2.left_stick_y);
            robot.setFlywheelsRPM(gamepad2.right_trigger);
            //robot.armLift.setPower(gamepad2.left_stick_y * robot.motorPower);

            if (gamepad1.x) robot.releaseIntake();
            if (gamepad1.y) robot.toggleRamp();

            telemetry.addData("Distance: ", robot.dist.getDistance(DistanceUnit.INCH));

            telemetry.update();
        }
        }

    }
