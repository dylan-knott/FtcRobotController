package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ProjectileSystems;

@TeleOp(name= "TeleOp", group= "TeleOp")
public class TeleOPMode extends LinearOpMode {
    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    ProjectileSystems shooter = new ProjectileSystems();
    APMecanumDrive drive = null;

    public void runOpMode() throws InterruptedException {

        //init for robot and shooter
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        shooter.initializeShooter(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        drive = robot.rrDrive;

        //Values to send to pose2d for driving
        final double stickDeadzone = 0.05;

        double strafe = 0;
        double forward = 0;
        double turn = 0;

        //Boolean values used in order to stop toggles from activating multiple times for a single button press
        boolean g1x_state = false;
        boolean g1y_state = false;


        waitForStart();

        while (opModeIsActive()) {

            //Movement code
            //0.971 tan 0.8x
            if (Math.abs(gamepad1.left_stick_y) < stickDeadzone) forward = 0; else forward = 0.971 * Math.tan(-gamepad1.left_stick_y * 0.8);
            if (Math.abs(gamepad1.left_stick_x) < stickDeadzone) strafe = 0; else strafe = 0.971 * Math.tan(-gamepad1.left_stick_x * 0.8);
            if (Math.abs(gamepad1.right_stick_x) < stickDeadzone) turn = 0; else turn = 0.971 * Math.tan(-gamepad1.right_stick_x * 0.8);


            drive.setWeightedDrivePower(
                    new Pose2d(
                            forward,
                            strafe,
                            turn
                    )
            );

            drive.update();
            //shooter.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Release Position", robot.intakeRelease.getPosition());


            //Gamepad 1  ***Drivetrain***
            if (gamepad1.x) robot.releaseIntake();
            if (gamepad1.right_bumper) robot.toggleClaw();

            //Gamepad 2  ***Gun and intake***
            robot.setIntake(gamepad2.right_stick_y);
            shooter.setFlywheel(gamepad2.right_trigger);
            shooter.indexer.setPosition((double)gamepad2.left_trigger);
            if (gamepad2.a) shooter.intakeBelt.setPower(1);
            else shooter.intakeBelt.setPower(0);

            //shooter.setFlywheelsRPM(gamepad2.right_trigger);

            if (gamepad2.dpad_up) robot.setArm(14);
            if (gamepad2.dpad_down) robot.setArm(0);

            if (isStopRequested()){
                robot.stop();
            }
            telemetry.update();
        }
    }
}