package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.APMecanumDrive;


public class LocalizedRobotDrive {
    //Proportional Processing values for distance to drive function
    private final double P_Forward = 0.05;
    private final double P_Strafe = 0.025;
    private final double P_Turn = 0.005;


    Telemetry telemetry = null;
    allianceColor teamColor = null;
    public APMecanumDrive rrDrive = null;

    //Hardware
    public DcMotor intake, armLift;
    public Servo clawServo, intakeRelease;
    public DistanceSensor dist = null;
    public DigitalChannel armLimit = null;

    //Default motor power levels for wheels
    public double motorPower = 0.8;
    public double intakePower = 1;
    public double armPower = 0.4;
    public int ringCount = 3;

    //Debug the error angle in order to get this value, sets the offset to which the robot will turn to meet the required degrees turned
    private final double TURNING_BUFFER = 0;

    //Up = false, Down = true
    private boolean rampState = false;
    //Open = false, Closed = true
    private boolean clawState = false;


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
        rampState = false;
        clawState = false;
        clawServo.setPosition(0);
        intakeRelease.setPosition(0);


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


    /***************************************FORWARD MOVEMENT***************************************/
    public void setMotorPowers(double mp) {
        rrDrive.setMotorPowers(mp, mp, mp, mp);
    }

    public void setMotorPowers(double lf, double lr, double rf, double rr) {
        rrDrive.setMotorPowers(lf, lr, rf, rr);
    }

    public void driveEncoder(double inches) {
        Trajectory driveTrajectory = null;
        if (inches >= 0) {
            driveTrajectory = rrDrive.trajectoryBuilder(new Pose2d())
                    .forward(inches)
                    .build();
        }
        else {
            driveTrajectory = rrDrive.trajectoryBuilder(new Pose2d())
                    .back(inches)
                    .build();
        }
        rrDrive.followTrajectory(driveTrajectory);
    }

    //Send this function a value of seconds to drive for and it will drive for that period
    public void driveTime(long time) throws InterruptedException {
        setMotorPowers(motorPower);
        Thread.sleep(time);
        setMotorPowers(0);
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

    public void strafeTime(int time, direction strafeDirection) throws InterruptedException {
        if (strafeDirection == direction.left) {
            setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);
        } else if (strafeDirection == direction.right) {
            setMotorPowers(motorPower, -motorPower, motorPower, -motorPower);
        }
        Thread.sleep(time);
        setMotorPowers(0);
    }

    /*******************************************STRAFING*******************************************/
    public void turn(double degrees) {
        rrDrive.turn(degrees * (Math.PI / 180));
    }

    public void turnAsync(double degrees) {
        rrDrive.turnAsync(degrees * (Math.PI / 180));
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

    public void setClaw(float position) {
            clawServo.setPosition(position);
    }

    public void setIntakeRelease(float degrees) {
        intakeRelease.setPosition(degrees / 280);
    }

    public void toggleClaw()
    {
        if (clawState) //claw is closed
        {
            clawServo.setPosition(0); //open claw
            clawState = false;
        }
        else
        {
            clawServo.setPosition(60.0f / 270); //close claw
            clawState = true;
        }
    }
    /*******************************************UTILITIES*******************************************/
    //Creating a clamp method for both floats and doubles, used to make sure motor power doesn't go above a certain power level as to saturate the motors
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
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


    //turning distance measurements into usable motor output via Proportional control)
    public void distanceToDrive(double forward, double right, double turn){

        double ForwardOut = forward * P_Forward;
        double RightOut = right * P_Strafe;
        double TurnOut = turn * P_Turn;
        mixDrive(ForwardOut, RightOut, TurnOut);
    }

}
