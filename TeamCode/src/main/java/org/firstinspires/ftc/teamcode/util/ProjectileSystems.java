package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Device;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;

import java.util.Timer;
import java.util.TimerTask;

public class ProjectileSystems
{
    //Hardware declaration, need color sensors
    public DcMotor intakeBelt;
    public DcMotorEx flywheel;
    public Servo indexer, reloader;


    public double flywheelPower = 1;
    public double intakePower = 1;

    Telemetry telemetry = null;
    LocalizedRobotDrive.allianceColor teamColor = null;

    private final double RPM_TO_TPS = 28.0f /60;

    Timer timer = new Timer();

    private enum Mode
    {
        HOLDING,
        FIRING,
        INTAKE,
        CHAMBER,
        PRIME
    }
    public Mode mode;

    public void initializeShooter(HardwareMap hardwareMap, Telemetry telem, LocalizedRobotDrive.allianceColor clr)
    {
        telemetry = telem;
        teamColor = clr;

        //Assigning HardwareMap
        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("flywheel_motor");
        intakeBelt = hardwareMap.dcMotor.get("conveyor");
        indexer = hardwareMap.servo.get("indexer");
        reloader = hardwareMap.servo.get("reloader");


        //Motor Initialization
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        indexer.setPosition(0);
        //reloader.setPosition(0);

        mode = Mode.HOLDING;
    }

    /*******************************************UNUSED*******************************************/

    public void fireRing(double inputSpeed) throws InterruptedException
    {

    }

    public void setFlywheel(double inputPower)
    {
        //Remap input to the max power
        double power = inputPower * flywheelPower;

        flywheel.setPower(power);
    }

    public void setFlywheelRPM(double power)
    {

        double flywheelAngularVelocity = 5 * RPM_TO_TPS * power;
        flywheel.setVelocity(flywheelAngularVelocity);
        telemetry.addData("Flywheel RPM: ", flywheelAngularVelocity);

    }
    public void setIndexer(double pos){
        indexer.setPosition(pos / 280.0f);
    }

    public void update()
    {
        //placeholder, need to figure out once build
        double firePOS = 180;
        double reloadPOS = 180;

        switch (mode)
        {
            case HOLDING:
                flywheel.setVelocity(5 * RPM_TO_TPS * 0);
                //indexer.setPosition(0);
                intakeBelt.setPower(0);
                //reloader.setPosition(0);
                break;

            case CHAMBER:
                //TODO: TEST IF THIS WORKS, MAY NOT COMPLETE. FIGURE OUT HOW TO OPEN, THEN WAIT FOR CLOSE TO START INTAKE MAY BE IDIOT
                //Used to specify how long to wait to close chambering servo TODO: Find correct value
                int reloaderDelay = 500;
                //reloader.setPosition(reloadPOS);
                //reloader.setPosition(0);
                TimerTask servoClose = new TimerTask() {
                    @Override
                    public void run() {
                        mode = Mode.PRIME;
                    }
                };
                timer.schedule(servoClose, reloaderDelay);
                break;

            case PRIME:
                //Used to specify how long to run belt for TODO: Find correct Value
                int beltDelay = 500;
                intakeBelt.setPower(1);
                TimerTask runBelt = new TimerTask() {
                    @Override
                    public void run() {
                        mode = Mode.HOLDING;
                    }
                };
                timer.schedule(runBelt, beltDelay);
                break;

            case INTAKE:
                intakeBelt.setPower(1);
        }

    }

}

