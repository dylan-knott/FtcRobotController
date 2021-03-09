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
    public final double ARM_REACH = 16;
    //Compensate for angled trajectory of ring out of the shooter
    public final double SHOOTER_ANGLE_ERROR = Math.toRadians(0);

    private Telemetry telemetry = null;
    private allianceColor teamColor = null;
    public APMecanumDrive rrDrive = null;
    private Timer timer = new Timer();

    //Hardware
    public DcMotor intake, armLift;
    public Servo clawServo, intakeRelease;
    public DistanceSensor dist = null;
    public DigitalChannel armLimit = null;

    //Default motor power levels for wheels
    public double motorPower = 0.8;
    public double intakePower = 1;
    public double armPower = 0.4;


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
        armLimit = hardwareMap.get(DigitalChannel.class, "arm_limit");

        //Expansion hub 1 servos
        clawServo = hardwareMap.servo.get("claw_servo");
        intakeRelease = hardwareMap.servo.get("intake_release");


        dist = hardwareMap.get(DistanceSensor.class, "distance");

        //Motor Initialization
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Initialize Arm
        initializeArm();

        //Initialize servos
        intakeRelease.setDirection(Servo.Direction.REVERSE);
        clawServo.setPosition(0);
        intakeRelease.setPosition(90 / 280f);


        //Sensor Initialization
        /*if (floorColor instanceof SwitchableLight) {
            ((SwitchableLight)floorColor).enableLight(false);
        }*/
        armLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    public void initializeArm()
    {
        //Set motor to run_without_encoder
        armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Run arm back until switch is activated
        armLift.setPower(-armPower);
        while (!armLimit.getState());
        //Set motor to run_to_position
        armLift.setTargetPosition(0);
        armLift.setMode((DcMotor.RunMode.RUN_TO_POSITION));
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

    /*******************************************STRAFING*******************************************/
    public void turn(double degrees) {
        rrDrive.turn(degrees * (Math.PI / 180));
    }

    /******************************************GAME FUNCTIONS********************************************/

    public void setArm(int posDegrees)
    {
        final double _ARM_RATIO_ = (60 * (20.0f / 15) * (15.0f / 10 ));
        armLift.setTargetPosition((int)(posDegrees * _ARM_RATIO_));
    }

    public void setIntake(float power) {
            intake.setPower(power * intakePower);
    }

    public void releaseIntake() {
        intakeRelease.setPosition(0 / 280f);
        if (!timerRunning) {
            timer.schedule(lockIntake, 2000L);
            timerRunning = true;
        }
    }

    public void setClaw(float position) {
            clawServo.setPosition(position);
    }

    public void setIntakeRelease(float position) {
        intakeRelease.setPosition(position / 280f);
    }

    public void toggleClaw()
    {
        if (clawServo.getPosition() == 0) //claw is closed
        {
            setClaw(275.0f / 280); //open claw
        }
        else
        {
            clawServo.setPosition(0); //close claw
        }
    }
    /*******************************************UTILITIES*******************************************/
    //Creating a clamp method for both floats and doubles, used to make sure motor power doesn't go above a certain power level as to saturate the motors
    private double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val));
    }

    TimerTask lockIntake = new TimerTask() {
        public void run() {
            setIntakeRelease(152);
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
