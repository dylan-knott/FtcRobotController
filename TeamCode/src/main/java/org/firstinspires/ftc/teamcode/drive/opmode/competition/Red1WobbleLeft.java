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

@Autonomous(name="Red 1 Wobble Left")
public class Red1WobbleLeft extends LinearOpMode {

    private final int INITIAL_TURN = -35;
    private final int SHOOTER_TIMEOUT_SECONDS = 7;

    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    ProjectileSystems shooter = new ProjectileSystems();
    TensorFlowRingIdentification tf = new TensorFlowRingIdentification();
    APMecanumDrive drive = null;
    Pose2d dropPose = null;

    public void runOpMode() throws InterruptedException {
        //init for robot and shooter
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        shooter.initializeShooter(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);
        tf.initObjectDetector(hardwareMap, telemetry);
        drive = robot.rrDrive;

        //Tell roadrunner where the robot is initially placed
        //Tell the shooter how many rings are in by default
        drive.setPoseEstimate(new Pose2d(-72 + robot.CHASSIS_LENGTH / 2 , -24 + robot.CHASSIS_WIDTH / 2 , 0));

        //Wait for start button to be pressed
        waitForStart();

        //Turn 15 degrees to look at ring stack
        drive.turn(Math.toRadians(INITIAL_TURN));

        //TODO: Look for Ring Stack
        char dropZone = tf.runDetect(2);
        tf.closeTfod();
        if (dropZone == 'c') dropPose = new Pose2d(52, -65 + robot.ARM_REACH, Math.toRadians(-90));
        else if (dropZone == 'b') dropPose = new Pose2d(36, -38 + robot.ARM_REACH, Math.toRadians(-90));
        else dropPose = new Pose2d(12, -64 + robot.ARM_REACH, Math.toRadians(-90));
        telemetry.addData("Drop Zone", dropZone);
        telemetry.update();
        //While the ring stack is being looked for, build the trajectory
        //This trajectory is for delivering the wobble goal, and driving up until rings are shot
        Trajectory trajA = drive.trajectoryBuilder(drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(INITIAL_TURN))))
                .splineTo(new Vector2d(-36, -12),0)
                .build();

        Trajectory trajAB = drive.trajectoryBuilder(trajA.end())
                    .splineToLinearHeading(dropPose, Math.toRadians(-90)) //Move to targeted drop zone
                    .build();


        Trajectory trajB = drive.trajectoryBuilder(trajAB.end(), true)
                .splineToLinearHeading(new Pose2d(-18, -18,drive.getRadiansToTarget(APMecanumDrive.Target.RED_TOWER)).minus(new Pose2d(0, 0, robot.SHOOTER_ANGLE_ERROR)), Math.toRadians(0))
                .build();


        Trajectory trajC = drive.trajectoryBuilder(trajB.end())
                .splineToLinearHeading(new Pose2d(4,  -12, 0), Math.toRadians(0))
                .build();


        //ROBOT ACTUALLY MOVES
        drive.followTrajectory(trajA);
        drive.followTrajectory(trajAB);
        robot.setArm(110);
        sleep(800);
        robot.setClaw(0);
        sleep(200);
        robot.setArm(0);
        drive.followTrajectory(trajB);
        //Shoot Rings
        long end = System.currentTimeMillis() + (int)(SHOOTER_TIMEOUT_SECONDS * 1000);
        shooter.fireRing(69, 3);
        while (shooter.getRingCount() > 0 && System.currentTimeMillis() < end) {
            shooter.update();
        }
        shooter.mode = ProjectileSystems.Mode.RESET;
        shooter.update();
        drive.followTrajectory(trajC);

        //At the end, store the pose
        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}
