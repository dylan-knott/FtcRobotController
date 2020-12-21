package org.firstinspires.ftc.teamcode;

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
            //Gamepad 1  ***Drivetrain***
            double forward = -gamepad1.left_stick_y; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;


            if (gamepad1.left_bumper) robot.motorPower = 0.2;
            else if (gamepad1.right_bumper) robot.motorPower= 0.15;
            else robot.motorPower = 0.8;
            //Wheel control
            robot.mixDrive(forward, strafe, rotate);

            //Gamepad 2  ***Gun and intake***
            robot.enableIntake(gamepad2.right_bumper);
            //robot.setFlywheels(gamepad2.left_trigger);
            robot.setFlywheelsRPM();
            robot.setIndexer(gamepad2.right_trigger);
            //robot.armLift.setPower(gamepad2.left_stick_y * robot.motorPower);



            telemetry.addData("Red: ", robot.floorColor.red());
            telemetry.addData("Green: ", robot.floorColor.green());
            telemetry.addData("Blue: ", robot.floorColor.blue());
            telemetry.addData("Distance: ", robot.dist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        }

    }
