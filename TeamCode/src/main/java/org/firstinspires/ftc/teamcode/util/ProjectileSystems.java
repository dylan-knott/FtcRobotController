package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;

import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

public class ProjectileSystems extends Thread
{
    //Hardware declaration, need color sensors
    public DcMotor intakeBelt;
    public DcMotorEx flywheel;
    public Servo indexer, deflector, deflector2;
    public RevColorSensorV3 dist;

    Telemetry telemetry = null;
    LocalizedRobotDrive.allianceColor teamColor = null;

    private final double RPM_TO_TPS = 28.0f /60;
    private final double TPS_TO_RPM = 60/28.0f;
    private final double NO_RING_DIST = 7.5;
    private final double ONE_RING_DIST = 6.1;
    private final double TWO_RING_DIST = 4;
    private int ringCount = 0;
    public boolean parentTerminated = false;
    public boolean readyToFire =true;
    public int lastfire =3;
    public boolean isAuto;
    double flywheelRPM = 4500;
    public int ringQueue;

    Timer timer = new Timer();

    public enum Mode
    {
        RESET,
        FIRING,
        IDLE,
    }

    public Mode mode;

    public void run() {
        while (true) {
            try {
                update();
            } catch (InterruptedException e) {
            }
        }
    }

    public void initializeShooter(HardwareMap hardwareMap, Telemetry telem, LocalizedRobotDrive.allianceColor clr)
    {
        telemetry = telem;
        teamColor = clr;

        //Assigning HardwareMap
        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("flywheel_motor");
        intakeBelt = hardwareMap.dcMotor.get("conveyor");
        indexer = hardwareMap.servo.get("indexer");
        deflector = hardwareMap.servo.get("deflector");
        deflector2 = hardwareMap.servo.get("deflector_2");
        deflector.setDirection(Servo.Direction.REVERSE);
        dist = hardwareMap.get(RevColorSensorV3.class, "distColor");


        //Motor Initialization
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeBelt.setDirection(DcMotor.Direction.REVERSE);

        //Servo Initialization
        deflector.setPosition(0);
        deflector2.setPosition(0);
        indexer.setPosition(0);

        //TODO: Hashmap
        //Distance to angle Hashmap Initalization
        //distToAngle4500.put();

        //Default robot init mode
        countRings();
        mode = Mode.IDLE;
    }

    public void fireRing(double degrees, boolean auto, int queue) {
        //TODO: connection of distance to deflector angle
        if (mode == Mode.IDLE) {
            setDeflector(degrees);
            isAuto = auto;
            setFlywheelRPM(flywheelRPM);
            ringQueue = queue;
            mode = Mode.FIRING;
        }
    }

    public void fireRing(double degrees, boolean auto) {
        //TODO: connection of distance to deflector angle
        if (mode == Mode.IDLE) {
            setDeflector(degrees);
            isAuto = auto;
            setFlywheelRPM(flywheelRPM);
            mode = Mode.FIRING;
        }
    }

    public void setDeflector(double position)
    {
        deflector2.setPosition(position / 270.0f);
        deflector.setPosition(position / 270.0f);
    }

    public void setFlywheelRPM(double rpm)
    {
        double flywheelAngularVelocity = RPM_TO_TPS * rpm;
        flywheel.setVelocity(flywheelAngularVelocity);
        //telemetry.addData("Flywheel RPM: ", flywheelAngularVelocity);
    }

    public int getRingCount (){
        countRings();
        return ringCount;
    }

    private void countRings() {
        double ringDist = dist.getDistance(DistanceUnit.CM);
        if (ringDist > NO_RING_DIST) {
            ringCount = 0;
        } else if (ringDist > ONE_RING_DIST) {
            ringCount = 1;
        } else if (ringDist > TWO_RING_DIST) {
            ringCount = 2;
        } else {
            ringCount = 3;
        }
    }

    public void update() throws InterruptedException
    {
        countRings();

        switch (mode){
            case IDLE:
                break;

            case RESET:
                indexer.setPosition(0);

                if(getRingCount() > 0 && ringQueue !=0 )
                {
                    readyToFire = true;
                    if (lastfire == getRingCount() && isAuto == true)
                        intakeBelt.setPower(1.f);
                    mode = Mode.FIRING;
                }
                else
                    {
                     setFlywheelRPM(0);
                     deflector.setPosition(0f);
                     deflector2.setPosition(0f);
                     intakeBelt.setPower(0);
                     readyToFire= true;
                     mode = Mode.IDLE;
                     }
                break;

            case FIRING:
                int fireDelay = 600;


                if (ringCount > 0 || isAuto == false)
                {

                    if (flywheel.getVelocity() * TPS_TO_RPM <= flywheelRPM + 40 && flywheel.getVelocity() * TPS_TO_RPM >= flywheelRPM - 10)
                    {
                        indexer.setPosition(30.0 / 280.0f);
                        TimerTask endFire = new TimerTask() {
                            @Override
                            public void run() { mode = Mode.RESET;
                            }
                        };

                        if (readyToFire == true) {
                            lastfire = getRingCount();
                            timer.schedule(endFire, fireDelay);
                            ringQueue--;
                            readyToFire = false;
                        }
                    }
                }
                else
                {
                    readyToFire = false;
                    mode= Mode.RESET;
                }
                break;
        }

        //telemetry.addData("In firing mode", flywheel.getVelocity() * TPS_TO_RPM);
        //telemetry.addData("Shooter Runtime", System.currentTimeMillis());
        //telemetry.update();
        if (parentTerminated) {
            Thread.currentThread().interrupt();
            return;
        }
    }
}

