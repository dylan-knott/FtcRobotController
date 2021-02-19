package org.firstinspires.ftc.teamcode.drive.opmode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;
import org.firstinspires.ftc.teamcode.RobotDrive;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ProjectileSystems;

@Autonomous(name="Blue 1 Wobble Left")
public class Blue1WobbleLeft extends LinearOpMode {
    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    ProjectileSystems shooter = new ProjectileSystems();
    APMecanumDrive drive = null;
    Pose2d dropPose = null;

    public void runOpMode() throws InterruptedException {
        //init for robot and shooter
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        shooter.initializeShooter(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        drive = robot.rrDrive;

        drive.setPoseEstimate(new Pose2d(-72 + robot.CHASSIS_LENGTH / 2 , 48 + robot.CHASSIS_WIDTH / 2 ,Math.toRadians(90)));


        waitForStart();

        drive.turn(Math.toRadians(30));

        //TODO: Look for Ring Stack
        if (true) dropPose = new Pose2d(48 - robot.ARM_REACH, 60);
        else if (true) dropPose = new Pose2d(36 - robot.ARM_REACH, 36);
        else /*no rings*/ new Pose2d(12 - robot.ARM_REACH, 60);
        //While the ring stack is being looked for, build the trajectory
        Trajectory trajA = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(dropPose, Math.toRadians(-90)) //Move to zone A
                .addDisplacementMarker(() -> { //Runs after the first spline is completed
                    //TODO: Drop wobble goal
                    robot.setArm(90);
                    robot.toggleClaw();
                })
                .build();

    }
}
