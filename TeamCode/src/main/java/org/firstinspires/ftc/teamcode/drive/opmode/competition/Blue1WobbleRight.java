package org.firstinspires.ftc.teamcode.drive.opmode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CV.TensorFlowRingIdentification;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.ProjectileSystems;

@Autonomous(name="Blue 1 Wobble Right")
public class Blue1WobbleRight extends LinearOpMode {

    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    ProjectileSystems shooter = new ProjectileSystems();
    TensorFlowRingIdentification tf = new TensorFlowRingIdentification();
    APMecanumDrive drive = null;
    Vector2d dropPoseA = new Vector2d(12, 64 - robot.ARM_REACH);
    Vector2d dropPoseB = new Vector2d(36, 38 - robot.ARM_REACH);
    Vector2d dropPoseC = new Vector2d(52, 65 - robot.ARM_REACH);
    Pose2d startPose = new Pose2d(-72 + robot.CHASSIS_LENGTH / 2 , 24 - robot.CHASSIS_WIDTH / 2,0 );

        //Build Trajectories

    public void runOpMode() throws InterruptedException {
        //init for robot and shooter
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        shooter.initializeShooter(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        shooter.setDaemon(true);
        tf.initObjectDetector(hardwareMap, telemetry);
        drive = robot.rrDrive;

        //Set up different trajectories based on where the ring stack determines the robot should go, they will be built ahead of time, and it will choose which to follow at run time
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-24, 10, Math.toRadians(90)), 0)
                .build();
        Trajectory traj1A = drive.trajectoryBuilder(traj0.end(), true)
                .splineToConstantHeading(dropPoseA, Math.toRadians(90)) //Move to targeted drop zone
                .build();
        Trajectory traj1B = drive.trajectoryBuilder(traj0.end(), true)
                .splineToConstantHeading(dropPoseB, Math.toRadians(90)) //Move to targeted drop zone
                .build();
        Trajectory traj1C = drive.trajectoryBuilder(traj0.end(), true)
                .splineToLinearHeading(new Pose2d(dropPoseC.getX(), dropPoseC.getY(), Math.toRadians(90)), Math.toRadians(75)) //Move to targeted drop zone
                .build();


        Trajectory traj2A = drive.trajectoryBuilder(traj1A.end(), true)
                .splineToLinearHeading(new Pose2d(-16, 10,drive.getRadiansToTarget(APMecanumDrive.Target.BLUE_POWERSHOT, -16, 10)).minus(new Pose2d(0, 0, robot.SHOOTER_ANGLE_ERROR)), Math.toRadians(180))
                .build();
        Trajectory traj2B = drive.trajectoryBuilder(traj1B.end(), true)
                .splineToLinearHeading(new Pose2d(-16, 10,drive.getRadiansToTarget(APMecanumDrive.Target.BLUE_POWERSHOT, -16, 10)).minus(new Pose2d(0, 0, robot.SHOOTER_ANGLE_ERROR)), Math.toRadians(180))
                .build();
        Trajectory traj2C = drive.trajectoryBuilder(traj1C.end(), true)
                .splineToLinearHeading(new Pose2d(-16, 10,drive.getRadiansToTarget(APMecanumDrive.Target.BLUE_POWERSHOT,-16, 10)).minus(new Pose2d(0, 0, robot.SHOOTER_ANGLE_ERROR)), Math.toRadians(180))
                .build();


        //Tell roadrunner where the robot is initially placed
        drive.setPoseEstimate(startPose);

        //Wait for start button to be pressed
        waitForStart();
        shooter.start();

        //Look for ring stack with tensorflow
        drive.followTrajectory(traj0);
        char dropZone =  tf.runDetect(1);
        tf.closeTfod();

        telemetry.addData("Drop Zone", dropZone);
        telemetry.update();

        if (dropZone == 'c') {
            //Run trajectory set c
            drive.followTrajectory(traj1C);
            robot.setArm(110);
            sleep(800);
            robot.setClaw(0);
            sleep(200);
            robot.setArm(0);
            drive.followTrajectory(traj2C);
        }
        else if (dropZone == 'b') {
            //Run trajectory set b
            drive.followTrajectory(traj1B);
            robot.setArm(110);
            sleep(800);
            robot.setClaw(0);
            sleep(200);
            robot.setArm(0);
            drive.followTrajectory(traj2B);
        }
        else {
            //Run trajectory set A
            drive.followTrajectory(traj1A);
            robot.setArm(110);
            sleep(800);
            robot.setClaw(0);
            sleep(200);
            robot.setArm(0);
            drive.followTrajectory(traj2A);
        }
        //Fan shots, aiming to separate power-shot poles for each shot
        shooter.fireRing(97, 1, true);
        while(shooter.getRingCount() > 2);
        drive.turn(Math.toRadians(-10));
        while(shooter.getRingCount() > 1);
        drive.turn(Math.toRadians(-10));
        //Build final path while the robot is shooting the last ring
        Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(12, 12, 0), Math.toRadians(0))
                .build();
        while(shooter.getRingCount() > 0);


        robot.setArm(0);
        drive.followTrajectory(traj3);

        //At the end, kill the shooting thread and store the pose
        //TODO: Kill shooting thread correctly
        shooter.parentTerminated = true;
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
