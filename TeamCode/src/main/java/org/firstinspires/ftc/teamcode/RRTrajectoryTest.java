package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;

@Disabled
public class RRTrajectoryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        LocalizedRobotDrive robot = new LocalizedRobotDrive(); //Robot Drive, handles servos and non-drivetrain specific functions as well as basic drivetrain movements
        APMecanumDrive rr = robot.rrDrive; //Direct roadrunner implementation. handles complex movement like splines and localized moves

        robot.initializeRobot(hardwareMap, telemetry, LocalizedRobotDrive.allianceColor.blue);

        waitForStart();

        Trajectory goForward = rr.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(100)
                .build();
        rr.followTrajectory(goForward);

        Trajectory lineToPosition = rr.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(new Vector2d(0, 0))
                .build();
        rr.followTrajectory(lineToPosition);

        Trajectory strafeLeft = rr.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(50)
                .build();
        rr.followTrajectory(strafeLeft);

        Trajectory splineToPosition = rr.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineTo(new Vector2d(50, 50), Math.toRadians(90))
                .build();
        rr.followTrajectory(splineToPosition);
    }
}
