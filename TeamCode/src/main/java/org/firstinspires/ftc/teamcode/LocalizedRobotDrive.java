package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;

import java.util.Timer;
import java.util.TimerTask;


public class LocalizedRobotDrive {
    //Width of the robot chassis
    public final double CHASSIS_WIDTH = 18;
    //Width of the robot length
    public final double CHASSIS_LENGTH = 18;
    //Compensate for reach of the wobble arm, measured from the center of the bot
    public final double ARM_REACH = 20;
    //Compensate for angled trajectory of ring out of the shooter
    public final double SHOOTER_ANGLE_ERROR = Math.toRadians(12);

    private Telemetry telemetry = null;
    private allianceColor teamColor = null;
    public APMecanumDrive rrDrive = null;
    private Timer timer = new Timer();

    //Hardware
    public DcMotor intake, armLift;
    public Servo clawServo, intakeRelease;
    public TouchSensor armLimit = null;

    //Default motor power levels for wheels
    public double motorPower = 0.8;
    public double intakePower = 0.7;


    private boolean timerRunning = false;


    public enum direction {
        left, right
    }

    public enum allianceColor {
        red, blue
    }

    //Assigning software objects to hardware, receives hardwareMap and telemetry objects from the op mode which calls it
    public void initializeRobot(HardwareMap hardwareMap, Telemetry telem, allianceColor clr) {
        rrDrive = new APMecanumDrive(hardwareMap);
        telemetry = telem;
        teamColor = clr;

        //Expansion hub 2 motors
        intake = hardwareMap.dcMotor.get("intake_motor");
        armLift = hardwareMap.dcMotor.get("arm_lift");

        //Expansion hub 2 sensors
        armLimit = hardwareMap.touchSensor.get("arm_limit");

        //Expansion hub 1 servos
        clawServo = hardwareMap.servo.get("claw_servo");
        intakeRelease = hardwareMap.servo.get("intake_release");

        //Motor Initialization
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        //Initialize Arm
        initializeArm();

        //Initialize servos
        intakeRelease.setDirection(Servo.Direction.REVERSE);
        intakeRelease.setPosition(90 / 280f);
        armLift.setDirection(DcMotor.Direction.REVERSE);

    }

    public void initializeArm()
    {
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        setClaw(90);
    }


    /*******************************************STRAFING*******************************************/
    //Send this function a time value as well as a direction (Ex: RobotDrive.direction.left) and it will strafe that direction for the specified amount of time
    public void strafeEncoder(double inches, direction d) {
        Trajectory strafeTrajectory = null;

        if (d == direction.right) {
            strafeTrajectory = rrDrive.trajectoryBuilder(new Pose2d())
                    .strafeRight(inches)
                    .build();
        }
        else {
            strafeTrajectory = rrDrive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(inches)
                    .build();
        }
        rrDrive.followTrajectory(strafeTrajectory);
    }

    /*******************************************TURNING*******************************************/
    public void turn(double degrees) {
        rrDrive.turn(degrees * (Math.PI / 180));
    }

    /******************************************GAME FUNCTIONS********************************************/

    public void setArm(int posDegrees)
    {
        //Need to figure out this ratio. Gear ratio is 120 to 1. Ticks per Revolution at the motor is 28
        final double _ARM_RATIO_ = (120.0f * 28) / 360.0f;
        armLift.setPower(1);
        armLift.setTargetPosition((int)(posDegrees * _ARM_RATIO_));
    }

    public void setIntake(double power) {
            intake.setPower(power * intakePower);
    }

    public void releaseIntake() {
        intakeRelease.setPosition(0 / 280f);
    }

    public void setClaw(double position) {
            clawServo.setPosition(position / 280.0f);
    }

    public void setIntakeRelease(double position) {
        intakeRelease.setPosition(position / 280.0f);
    }

    public void toggleClaw()
    {
        if (clawServo.getPosition() == 0) //claw is open
        {
            setClaw(90.0f / 280); //close claw
        }
        else
        {
            clawServo.setPosition(0); //open claw
        }
    }
    /*******************************************UTILITIES*******************************************/
    //Creating a clamp method for both floats and doubles, used to make sure motor power doesn't go above a certain power level as to saturate the motors
    private double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val));
    }

    TimerTask lockIntake = new TimerTask() {
        public void run() {
            setIntakeRelease(90);
            timerRunning = false;
        }
    };

    private float clamp(float val, float min, float max) { return Math.max(min, Math.min(max, val));
    }

    private int clamp(int val, int min, int max) { return Math.max(min, Math.min(max, val));
    }

    private double getEstimatedX() { return rrDrive.getPoseEstimate().getX(); }
    private double getEstimatedY() { return rrDrive.getPoseEstimate().getY(); }
    private double getEstimatedHeading() {return rrDrive.getPoseEstimate().getHeading(); }

    //Returns the current heading of the robot when it is called, takes reading from the IMU gyroscope
    private double getHeading() {
        return rrDrive.getExternalHeading();
    }

    //Driving function that accepts three values for forward, strafe, and rotate, then mixes those values into 4 usable motor power outputs and sends to the motors.
    public void mixDrive(double forward, double strafe, double rotate) {
        double frontLeftSpeed = clamp((forward + strafe + rotate), -motorPower, motorPower);
        double frontRightSpeed = clamp((forward - strafe - rotate), -motorPower, motorPower);
        double backLeftSpeed = clamp((forward - strafe + rotate ), -motorPower, motorPower);
        double backRightSpeed = clamp((forward + strafe - rotate), -motorPower, motorPower);

        rrDrive.setMotorPowers(frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
    }

  public void stop() {
        rrDrive.setMotorPowers(0, 0, 0, 0);
        timer.cancel();
        timer.purge();

    }

}
