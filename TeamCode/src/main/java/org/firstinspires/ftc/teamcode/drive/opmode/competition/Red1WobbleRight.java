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

@Autonomous(name="Red 1 Wobble Right")
public class Red1WobbleRight extends LinearOpMode {

    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    ProjectileSystems shooter = new ProjectileSystems();
    TensorFlowRingIdentification tf = new TensorFlowRingIdentification();
    APMecanumDrive drive = null;
    Vector2d dropPoseA = new Vector2d(10 - robot.ARM_REACH, -55);
    Vector2d dropPoseB = new Vector2d(38 - robot.ARM_REACH, -35);
    Vector2d dropPoseC = new Vector2d(56 - robot.ARM_REACH, -53);
    Vector2d shootPose = new Vector2d(-11, -54);
    Pose2d startPose = new Pose2d(-72 + robot.CHASSIS_LENGTH / 2 , -48 - robot.CHASSIS_WIDTH / 2,0 );

        //Build Trajectories

    public void runOpMode() throws InterruptedException {
        //init for robot and shooter
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.red);
        shooter.initializeShooter(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.red);
        shooter.setDaemon(true);
        tf.initObjectDetector(hardwareMap, telemetry);
        drive = robot.rrDrive;

        //Set up different trajectories based on where the ring stack determines the robot should go, they will be built ahead of time, and it will choose which to follow at run time
        //Trajectory to drive to look at rings
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-55, -55), Math.toRadians(35))
                .build();
        //Trajectories for each ring drop zone
        Trajectory traj1A = drive.trajectoryBuilder(traj0.end(),true)
                .lineToLinearHeading(new Pose2d(dropPoseA, 0)) //Move to targeted drop zone
                .build();
        Trajectory traj1B = drive.trajectoryBuilder(traj0.end(), true)
                .lineToLinearHeading(new Pose2d(dropPoseB, 0)) //Move to targeted drop zone
                .build();
        Trajectory traj1C = drive.trajectoryBuilder(traj0.end(), true)
                .lineToLinearHeading(new Pose2d(dropPoseC, 0)) //Move to targeted drop zone
                .build();

        //Trajectories to approach the shooting position starting at each drop zone
        Trajectory traj2A = drive.trajectoryBuilder(traj1A.end())
                .lineToLinearHeading(new Pose2d(shootPose ,drive.getRadiansToTarget(APMecanumDrive.Target.RED_TOWER, shootPose.getX(), shootPose.getY())).plus(new Pose2d(0, 0, Math.toRadians(-10))))
                .build();
        Trajectory traj2B = drive.trajectoryBuilder(traj1B.end())
                .lineToLinearHeading(new Pose2d(shootPose,drive.getRadiansToTarget(APMecanumDrive.Target.RED_TOWER, shootPose.getX(), shootPose.getY())).plus(new Pose2d(0, 0, Math.toRadians(-10))))
                .build();
        Trajectory traj2C = drive.trajectoryBuilder(traj1C.end())
                .lineToLinearHeading(new Pose2d(shootPose,drive.getRadiansToTarget(APMecanumDrive.Target.RED_TOWER, shootPose.getX(), shootPose.getY())).plus(new Pose2d(0, 0, Math.toRadians(-10))))
                .build();


        //Tell roadrunner where the robot is initially placed
        drive.setPoseEstimate(startPose);

        //Wait for start button to be pressed
        waitForStart();
        shooter.start();

        //Look for ring stack with tensorflow
        drive.followTrajectory(traj0);
        char dropZone =  tf.runDetect(5);
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
        shooter.fireRing(110, true);
        Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(12, -36, 0))
                .build();
        while(shooter.getRingCount() > 0);

        sleep(100);
        robot.setArm(0);
        drive.followTrajectory(traj3);

        //At the end, kill the shooting thread and store the pose
        //TODO: Kill shooting thread correctly
        shooter.parentTerminated = true;
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
