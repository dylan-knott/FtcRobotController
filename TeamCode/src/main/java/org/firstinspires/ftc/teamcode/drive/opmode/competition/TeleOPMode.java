package org.firstinspires.ftc.teamcode.drive.opmode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.ProjectileSystems;

@TeleOp(name= "TeleOp", group= "TeleOp")
public class TeleOPMode extends LinearOpMode {
    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    ProjectileSystems shooter = new ProjectileSystems();
    APMecanumDrive drive = null;

    public void runOpMode() throws InterruptedException {

        shooter.setDaemon(true);
        if (shooter.isAlive()) {
            shooter.stop();
        }

        //init for robot and shooter
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        shooter.initializeShooter(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        drive = robot.rrDrive;

        //Values to send to pose2d for driving
        final double stickDeadzone = 0.05;

        double strafe = 0;
        double forward = 0;
        double turn = 0;

        shooter.start();
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        robot.setIntakeRelease(0);
        shooter.mode = ProjectileSystems.Mode.IDLE;
        while (opModeIsActive()) {

            //Movement code
            //0.971 tan 0.8x
            if (Math.abs(gamepad1.left_stick_y) < stickDeadzone) forward = 0; else forward = 0.971 * Math.tan(-gamepad1.left_stick_y * 0.8);
            if (Math.abs(gamepad1.left_stick_x) < stickDeadzone) strafe = 0; else strafe = 0.971 * Math.tan(-gamepad1.left_stick_x * 0.8);
            if (Math.abs(gamepad1.right_stick_x) < stickDeadzone) turn = 0; else turn = 0.971 * Math.tan(-gamepad1.right_stick_x * 0.8);

            //if (gamepad1.left_bumper)

            drive.setWeightedDrivePower(
                    new Pose2d(
                            forward,
                            strafe,
                            turn
                    )
            );

            drive.update();


            //Gamepad 1  ***Drivetrain***
            if (gamepad1.x) robot.releaseIntake();
            if (gamepad1.right_bumper) robot.setClaw(90);
            if (gamepad1.left_bumper) robot.setClaw(0);
            if (gamepad1.dpad_up) robot.setArm(110);
            if (gamepad1.dpad_down) robot.setArm(0);


            //Gamepad 2  ***Gun and intake***
            robot.setIntake(gamepad2.right_stick_y * robot.intakePower);
            shooter.intakeBelt.setPower(-gamepad2.left_stick_y);


            //testing
            if(gamepad2.y) shooter.fireRing(97, 1, false);

            if (isStopRequested()){
                robot.stop();
                shooter.parentTerminated = true;
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}