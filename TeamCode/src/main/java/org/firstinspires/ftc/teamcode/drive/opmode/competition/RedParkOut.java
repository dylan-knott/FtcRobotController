package org.firstinspires.ftc.teamcode.drive.opmode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CV.TensorFlowRingIdentification;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Lights;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.ProjectileSystems;

@Autonomous(name="Red Park Out")
public class RedParkOut extends LinearOpMode {

    LocalizedRobotDrive robot = new LocalizedRobotDrive();
    ProjectileSystems shooter = new ProjectileSystems();
    Lights lights = new Lights();
    TensorFlowRingIdentification tf = new TensorFlowRingIdentification();
    APMecanumDrive drive = null;
    Pose2d startPose = new Pose2d(-72 + robot.CHASSIS_LENGTH / 2 , 48 - robot.CHASSIS_WIDTH / 2,0 );

        //Build Trajectories

    public void runOpMode() throws InterruptedException {
        //init for robot and shooter
        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.red);
        lights.initializeLights(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.red);
        shooter.initializeShooter(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.red);
        shooter.setDaemon(true);
        tf.initObjectDetector(hardwareMap, telemetry);
        drive = robot.rrDrive;

        //Set up different trajectories based on where the ring stack determines the robot should go, they will be built ahead of time, and it will choose which to follow at run time
        //Trajectory to drive to look at rings
        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(12, 12, 0), 0)
                .build();

        //Tell roadrunner where the robot is initially placed
        drive.setPoseEstimate(startPose);

        //Wait for start button to be pressed
        waitForStart();
        shooter.start();

        //Look for ring stack with tensorflow
        sleep(25000);
        lights.setLights(Lights.Mode.park);
        drive.followTrajectory(traj0);

        //At the end, kill the shooting thread and store the pose
        //TODO: Kill shooting thread correctly
        shooter.parentTerminated = true;
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
