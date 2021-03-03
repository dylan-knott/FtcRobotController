package org.firstinspires.ftc.teamcode.drive.opmode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.CV.TensorFlowRingIdentification;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;
import org.firstinspires.ftc.teamcode.RobotDrive;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ProjectileSystems;

@Autonomous(name="Blue 1 Wobble Left")
public class Blue1WobbleLeft extends LinearOpMode {

    private final int INITIAL_TURN = 15;

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

        drive.setPoseEstimate(new Pose2d(-72 + robot.CHASSIS_LENGTH / 2 , 48 + robot.CHASSIS_WIDTH / 2 , 0));


        waitForStart();

        drive.turn(Math.toRadians(INITIAL_TURN));

        //TODO: Look for Ring Stack
        char dropZone = tf.runDetect(5);
        if (dropZone == 'c') dropPose = new Pose2d(48 - robot.ARM_REACH, 60, 0);
        else if (dropZone == 'b') dropPose = new Pose2d(36 - robot.ARM_REACH, 36, 0);
        else new Pose2d(12 - robot.ARM_REACH, 60, 0);
        //While the ring stack is being looked for, build the trajectory
        //This trajectory is for delivering the wobble goal, and driving up until rings are shot
        Trajectory trajA = drive.trajectoryBuilder(drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(INITIAL_TURN))))
                .splineToLinearHeading(dropPose, Math.toRadians(-90)) //Move to zone A
                .addDisplacementMarker(() -> { //Runs after the first spline is completed
                    //DO NOT CALL ANY SLEEP FUNCTIONS/FREEZE INTERPRETER INSIDE OF DISPLACEMENT MARKERS.
                    //TODO: Drop wobble goal
                    robot.setArm(90);
                    robot.setClaw(0);
                })
                .splineToLinearHeading(new Pose2d(-18, 60, drive.getRadiansToTarget(APMecanumDrive.Target.BLUE_TOWER)), Math.toRadians(-90))
                .build();

        Trajectory trajB = drive.trajectoryBuilder(trajA.end())
                .splineToConstantHeading(new Vector2d(12 - robot.ARM_REACH  + 3, 60), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    //Set Arm out to reach over the
                    robot.setArm(90);
                })
                .build();

        drive.followTrajectory(trajA);
        //TODO: Shoot Rings (for now, a sleep will emulate the time to shoot 3 rings
        sleep(2000);
        //TODO: Intake additional rings?

    }
}
