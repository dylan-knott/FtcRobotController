package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class RobotDrive {
    //Proportional Processing values for distance to drive function
    private final double P_Forward = 0.05;
    private final double P_Strafe = 0.025;
    private final double P_Turn = 0.005;

    Telemetry telemetry = null;
    allianceColor teamColor = null;

    //Proportional Value used in self-correcting gyro code for encoder driving
    private final double TURN_P = 0.005;

    //PID utilities for GyroTurn function
    private final double GYRO_P = 0.01;
    private final double wheelDiameter = 3.93701;

    //Servo function constants
    private final double INDEXER_SPEED = 0.7;


    //Hardware
    public DcMotorEx leftFront, leftRear, rightFront, rightRear = null;
    private final DcMotorEx[] motors = {leftFront, leftRear, rightFront, rightRear};
    public DcMotor intake;
    public DcMotor leftFlywheel, rightFlywheel;
    public Servo clawServo;
    public CRServo intakeBelt;
    private BNO055IMU imu = null;
    public DistanceSensor dist = null;
    public ColorSensor floorColor = null;
    public ColorSensor intakeColor = null;

    //Default motor power levels for wheels
    public double motorPower = 0.6;
    public double flywheelSpeed = 1;
    public double intakeSpeed = 1;
    public int ringCount = 3;

    //Debug the error angle in order to get this value, sets the offset to which the robot will turn to meet the required degrees turned
     private final double TURNING_BUFFER = 0;

    public enum direction {
        left, right
    }

    public enum allianceColor {
        red, blue
    }

    //Assigning software objects to hardware, receives hardwareMap and telemetry objects from the op mode which calls it
    public void initializeRobot(HardwareMap hardwareMap, Telemetry telem, allianceColor clr) {
        telemetry = telem;
        teamColor = clr;

        //Initialize hardware from hardware map
        //Expansion hub 1 motors
        leftFront = (DcMotorEx)hardwareMap.dcMotor.get("front_left_motor");
        rightFront = (DcMotorEx)hardwareMap.dcMotor.get("front_right_motor");
        leftRear = (DcMotorEx)hardwareMap.dcMotor.get("back_left_motor");
        rightRear = (DcMotorEx)hardwareMap.dcMotor.get("back_right_motor");
       ///Expansion hub 2 motors
        intake = hardwareMap.dcMotor.get("intake_motor");
        leftFlywheel= hardwareMap.dcMotor.get("left_flywheel_motor");
        rightFlywheel = hardwareMap.dcMotor.get("right_flywheel_motor");

        //Expansion hub 1 servos
        clawServo = hardwareMap.servo.get("claw_servo");
        intakeBelt = hardwareMap.crservo.get("intake_servo");

        //Expansion hub 1 sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        dist = hardwareMap.get(DistanceSensor.class, "distance");
        floorColor = hardwareMap.get(ColorSensor.class, "floor_color");
        intakeColor = hardwareMap.get(ColorSensor.class, "intake_color");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sensor Initialization
        if (floorColor instanceof SwitchableLight) {
            ((SwitchableLight)floorColor).enableLight(false);
        }

        //Sensor Initialization
        if (intakeColor instanceof SwitchableLight) {
            ((SwitchableLight)intakeColor).enableLight(false);
        }


        //Motor initialization
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setPositionPIDFCoefficients(5);
        rightRear.setPositionPIDFCoefficients(5);
        leftFront.setPositionPIDFCoefficients(5);
        leftRear.setPositionPIDFCoefficients(5);


        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

    }


    /***************************************FORWARD MOVEMENT***************************************/
    public void setMotors(double motorPower) {
        for (DcMotor motor: motors) motor.setPower(motorPower);
    }

    //Send this function a value of seconds to drive for and it will drive for that period
    public void driveTime(long time) throws InterruptedException {
        for (DcMotor motor: motors) motor.setPower(motorPower);
        Thread.sleep(time);
        for (DcMotor motor: motors) motor.setPower(0);

        telemetry.addData("Rear Left", leftRear.getCurrentPosition());
        telemetry.addData("Front Left", leftFront.getCurrentPosition());
        telemetry.addData("Front Right", rightFront.getCurrentPosition());
        telemetry.addData("Rear Right", rightRear.getCurrentPosition());
        telemetry.update();

    }

    //Send this function a number of inches and it will drive that distance using the encoders on the motors.
    public void driveEncoder(double Inches) {
        float initialHeading = getHeading();
        int encoderTicks = 0;
        if (Inches > 0) encoderTicks = (int)(480 * (float)((Inches - 1) / (wheelDiameter * Math.PI)));
        else if(Inches < 0) encoderTicks = (int)(480 * (float)((Inches + 1) / (wheelDiameter * Math.PI)));
         for (DcMotor motor: motors) {
             motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             motor.setTargetPosition(encoderTicks);
             motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }

         for (DcMotor motor: motors) {
             motor.setPower(motorPower);
         }

        while (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) > 30) {
            //wait until the motors are done running
           if (Math.abs((initialHeading - getHeading()) % 360) > 1) {
               double degreesCorrect = (initialHeading - getHeading()) % 360;
               double motorCorrect = clamp(degreesCorrect * GYRO_P, -.4, .4);
               leftFront.setPower(motorPower - motorCorrect);
               leftRear.setPower(motorPower - motorCorrect);
               rightFront.setPower(motorPower + motorCorrect);
               rightRear.setPower(motorPower + motorCorrect);
           }
                telemetry.addData("Rear Left", leftRear.getCurrentPosition());
                telemetry.addData("Front Left", leftFront.getCurrentPosition());
                telemetry.addData("Front Right", rightFront.getCurrentPosition());
                telemetry.addData("Rear Right", rightRear.getCurrentPosition());
                telemetry.update();
        }
        for (DcMotor motor : motors)
            motor.setPower(0);

        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    /*******************************************STRAFING*******************************************/
    //Send this function a time value as well as a direction (Ex: RobotDrive.direction.left) and it will strafe that direction for the specified amount of time
    public void strafeTime(int time, direction strafeDirection) throws InterruptedException {
        if (strafeDirection == direction.left) {
            leftFront.setPower(-1 * motorPower);
            leftRear.setPower(motorPower);
            rightRear.setPower(-1 * motorPower);
            rightFront.setPower(motorPower);
        } else if (strafeDirection == direction.right) {
            leftFront.setPower(motorPower);
            leftRear.setPower(-1 * motorPower);
            rightRear.setPower(motorPower);
            rightFront.setPower(-1 * motorPower);
        }
        Thread.sleep(time);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    //Send this function a distance in inches as well as a direction (Ex: RobotDrive.direction.right) and it will strafe that direction for the specified distance
    public void strafeEncoder(double Inches, direction direction) {
        float initialHeading = getHeading();

        int encoderTicks = (int)(480 * (float)(Inches / (wheelDiameter * Math.PI)));
        if (direction == RobotDrive.direction.left) encoderTicks *= -1;
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(encoderTicks);
        rightFront.setTargetPosition(-1 * encoderTicks);
        rightRear.setTargetPosition(encoderTicks);
        leftRear.setTargetPosition(-1 *encoderTicks);
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        leftFront.setPower(motorPower - 0.1);
        rightFront.setPower(motorPower - 0.1);
        leftRear.setPower(motorPower + 0.1);
        rightRear.setPower(motorPower + 0.1);

        while (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) > 30) {
            //wait until the motors are done running

            //Gyro Stabilize forward movement
            if (Math.abs((initialHeading - getHeading()) % 360) > 1) {
                double degreesCorrect = (initialHeading - getHeading()) % 360;
                double motorCorrect = clamp(degreesCorrect * GYRO_P, -.8, .8);
                leftFront.setPower(motorPower - motorCorrect);
                leftRear.setPower(motorPower - motorCorrect);
                rightFront.setPower(motorPower + motorCorrect);
                rightRear.setPower(motorPower + motorCorrect);
            }
            telemetry.addData("Front Left: ", leftFront.getCurrentPosition());
            telemetry.addData("Front Right: ", rightFront.getCurrentPosition());
            telemetry.addData("Back Left: ", leftRear.getCurrentPosition());
            telemetry.addData("Back Right: ", rightRear.getCurrentPosition());
            telemetry.update();
        }

        //Once motors are done running
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }


    /*******************************************TURNING********************************************/
    //Send this a value of degrees to turn, positive value turns right and negative value turns left, uses IMU gyroscope to precisely turn that distance
    public void gyroTurn(double degrees, long currentMillis) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target_angle = getHeading() - degrees;
        if (degrees < 0) {target_angle += TURNING_BUFFER;} else if (degrees > 0) {target_angle -= TURNING_BUFFER;}
        while (Math.abs((target_angle - getHeading()) % 360) > 3) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute turning error
            double motor_output = clamp(error_degrees * TURN_P, -.9, .9); // Get Correction of error
            //Send corresponding value to motors
            leftFront.setPower(-1 * motor_output);
            leftRear.setPower(-1 * motor_output);
            rightFront.setPower(motor_output);
            rightRear.setPower(motor_output);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Target Angle : ", target_angle - TURNING_BUFFER);
            telemetry.addData("Current Heading : ", String.format(Locale.getDefault(), "%.1f", angles.firstAngle * -1));
            telemetry.update();
        }

        telemetry.addData("Error Degrees: ", Math.abs(target_angle - angles.firstAngle) % 360);
        telemetry.update();
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void gyroTurn(double degrees, double motorClamp, long currentMillis) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target_angle = getHeading() - degrees;
        if (degrees < 0) {target_angle += TURNING_BUFFER;} else if (degrees > 0) {target_angle -= TURNING_BUFFER;}
        while (Math.abs((target_angle - getHeading()) % 360) > 3) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute turning error
            double motor_output = clamp(error_degrees * TURN_P, -motorClamp, motorClamp); // Get Correction of error
            //Send corresponding value to motors
            leftFront.setPower(-1 * motor_output);
            leftRear.setPower(-1 * motor_output);
            rightFront.setPower(motor_output);
            rightRear.setPower(motor_output);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Target Angle : ", target_angle - TURNING_BUFFER);
            telemetry.addData("Current Heading : ", String.format(Locale.getDefault(), "%.1f", angles.firstAngle * -1));
            telemetry.update();
        }

        telemetry.addData("Error Degrees: ", Math.abs(target_angle - angles.firstAngle) % 360);
        telemetry.update();
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }


    /******************************************GAME FUNCTIONS********************************************/

   public void setFlywheels(double inputSpeed) {
       //Remap input to respect the max speed
       double power = inputSpeed * flywheelSpeed;

       rightFlywheel.setPower(power);
       leftFlywheel.setPower(power);
   }

   public void enableIntake(boolean state) {
       if (state) //Intake is enabled
       {
           intake.setPower(intakeSpeed);
           //TODO: Controlling stacking and inner intake system


       } else { // Intake is disabled
           intake.setPower(0);
           //TODO: Controlling stacking and inner intake system


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

    //For debugging, displays current encoder values of each wheel
    public void getEncoderVals() {
        telemetry.addData("Encoders (LF, RF, LR, RR)", "%d %d %d %d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());
        telemetry.update();
    }

    //Returns the current heading of the robot when it is called, takes reading from the IMU gyroscope
    private float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    private void resetEncoders() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        telemetry.addLine("Encoders Reset!");
        telemetry.update();
    }

    //Driving function that accepts three values for forward, strafe, and rotate, then mixes those values into 4 usable motor power outputs and sends to the motors.
    public void mixDrive(double forward, double strafe, double rotate) {
        double frontLeftSpeed = clamp((forward + strafe + rotate), -motorPower, motorPower);
        double frontRightSpeed = clamp((forward - strafe - rotate), -motorPower, motorPower);
        double backLeftSpeed = clamp((forward - strafe + rotate ), -motorPower, motorPower);
        double backRightSpeed = clamp((forward + strafe - rotate), -motorPower, motorPower);

        leftFront.setPower(frontLeftSpeed);
        rightFront.setPower(frontRightSpeed);
        leftRear.setPower(backLeftSpeed);
        rightRear.setPower(backRightSpeed);
    }

    //turning distance measurements into usable motor output via Proportional control)
    public void distanceToDrive(double forward, double right, double turn){

        double ForwardOut = forward * P_Forward;
        double RightOut = right * P_Strafe;
        double TurnOut = turn * P_Turn;
        mixDrive(ForwardOut, RightOut, TurnOut);
    }

}
