package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ProjectileSystems;

@TeleOp(name= "TeleOp", group= "TeleOp")
public class TeleOPMode extends LinearOpMode {
    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    ProjectileSystems shooter = new ProjectileSystems();
    APMecanumDrive drive = null;

    public void runOpMode() throws InterruptedException{

        //init for robot and shooter
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        shooter.initializeShooter(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        drive = robot.rrDrive;

        //Values to send to pose2d for driving
        double forward = 0;
        double fExpo = 1.96;
        double strafe = 0;
        double sExpo = 1.96;
        double turn = 0;
        double tExpo = 1.96;

        //Boolean values used in order to stop toggles from activating multiple times for a single button press
        boolean g1x_state = false;
        boolean g1y_state = false;


        waitForStart();

        while (opModeIsActive()) {

            //Movement code
            forward = Math.pow(-gamepad1.left_stick_y, fExpo);
            strafe = Math.pow(-gamepad1.left_stick_x, sExpo);
            turn = Math.pow(-gamepad1.right_stick_x, tExpo);


            drive.setWeightedDrivePower(
                    new Pose2d(
                            forward,
                            strafe,
                            turn
                    )
            );

            drive.update();
            shooter.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();



            //Gamepad 1  ***Drivetrain***
            if (gamepad1.x){
                if (!g1x_state) {
                    robot.setIntakeRelease(90);
                    g1x_state = true;
                }
            } else g1x_state = false;


<<<<<<< Updated upstream
=======
            if (gamepad1.y){
                if (!g1y_state) {
                    robot.toggleRamp();
                    g1y_state = true;
                }
            } else g1y_state = false;

            robot.setClaw(gamepad1.right_trigger);



>>>>>>> Stashed changes
            //Gamepad 2  ***Gun and intake***
            robot.setIntake(gamepad2.right_stick_y);
            //shooter.setFlywheelsRPM(gamepad2.right_trigger);

            if(gamepad2.dpad_up) robot.setArm(14);
            if(gamepad2.dpad_down) robot.setArm(0);
            if(gamepad2.right_bumper) robot.toggleClaw();
        }
        }
    }
