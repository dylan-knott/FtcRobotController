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
    public Servo indexer, reloader, deflector;
    public RevColorSensorV3 dist;


    public double flywheelPower = 1;
    public double intakePower = 1;

    Telemetry telemetry = null;
    LocalizedRobotDrive.allianceColor teamColor = null;

    private final double RPM_TO_TPS = 28.0f /60;
    private final double TPS_TO_RPM = 60/28.0f;
    private final double NO_RING_DIST = 7.5;
    private final double ONE_RING_DIST = 6.1;
    private final double TWO_RING_DIST = 4;
    private int ringCount = 0;
    private int queuedRings;
    public boolean parentTerminated = false;

    private Map<Float, Integer> distToAngle4500  = new HashMap<>();;

    public void run() {
        while (true) {
            try {
                update();
            } catch (InterruptedException e) {
            }
        }
    }

    Timer timer = new Timer();

    public enum Mode
    {
        RESET,
        FIRING,
        INTAKE,
        CHAMBER,
        PRIME,
        IDLE,
    }
    public Mode mode;


    public boolean readyToFire =true;
    public void initializeShooter(HardwareMap hardwareMap, Telemetry telem, LocalizedRobotDrive.allianceColor clr)
    {
        telemetry = telem;
        teamColor = clr;

        //Assigning HardwareMap
        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("flywheel_motor");
        intakeBelt = hardwareMap.dcMotor.get("conveyor");
        indexer = hardwareMap.servo.get("indexer");
        deflector = hardwareMap.servo.get("deflector");
        deflector.setDirection(Servo.Direction.REVERSE);
        reloader = hardwareMap.servo.get("reloader");
        dist = hardwareMap.get(RevColorSensorV3.class, "distColor");


        //Motor Initialization
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeBelt.setDirection(DcMotor.Direction.REVERSE);


        //Servo Initialization
        deflector.setPosition(0);
        indexer.setPosition(0);
        reloader.setPosition(0);

        //TODO: Hashmap
        //Distance to angle Hashmap Initalization
        //distToAngle4500.put();

        //Default robot init mode
        countRings();
        mode = Mode.IDLE;
    }

    /*******************************************UNUSED*******************************************/

    public void fireRing(double degrees, int numQueued)
    {
        //TODO: connection of distance to deflector angle
        if (mode == Mode.IDLE) {
            setDeflector(degrees);
            queuedRings += numQueued;
            if (queuedRings > 3) {
                queuedRings = 3;
            }
            mode = ProjectileSystems.Mode.FIRING;
        }
    }

    public void setDeflector(double position){
        deflector.setPosition(position / 165.0f);
    }

    public void setFlywheelRPM(double rpm)
    {

        double flywheelAngularVelocity = RPM_TO_TPS * rpm;
        flywheel.setVelocity(flywheelAngularVelocity);
        telemetry.addData("Flywheel RPM: ", flywheelAngularVelocity);

    }
    public int getRingCount (){
        countRings();
        return ringCount;
    }

    //Returns if the shooter is Idle or busy
    public boolean isBusy() {
        if (mode == Mode.IDLE) return false;
        else return true;
    }

    //Code for ring count
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

    public void update() throws InterruptedException {
        //placeholder, need to figure out once built
        double firePOS = 180;
        double reloadPOS = 180;

        //telemetry.addData("Rings Queued", getRingCount());
        countRings();

        switch (mode) {
            case RESET:
                //set all moving system to default position/off
                if (getRingCount() <= 0) {
                    readyToFire = true;
                    setFlywheelRPM(0);
                    deflector.setPosition(0);
                }
                indexer.setPosition(0);
                intakeBelt.setPower(0);//Set Velo instead?
                reloader.setPosition(0);
                if (getRingCount() > 0) {
                    readyToFire = true;
                    //queuedRings--;
                    mode = Mode.FIRING;

                } else {
                    mode = Mode.IDLE;
                    break;
                }

            case FIRING:
                //Fire the current ring, and set mode to reset

                //time alloted for spinup before firing-may be remove
                int timeTF = 500;
                double flywheelRPM = 4500;
                //Turn on flywheel to set RPM -Find correct rpm, make it changeable?
                if (getRingCount() > 0) {
                    setFlywheelRPM(flywheelRPM);


                    //Mix of new untested code(Get velo statement) and old(setting flywheel power to full, wait timeTF, then move indexer, and reset
                    if (flywheel.getVelocity() * TPS_TO_RPM <= flywheelRPM + 40 && flywheel.getVelocity() * TPS_TO_RPM >= flywheelRPM - 10) {
                        //sleep(100);
                        indexer.setPosition(30.0 / 280.0f);
                        TimerTask endFire = new TimerTask() {
                            @Override
                            public void run() {
                                mode = Mode.RESET;
                            }
                        };
                        if (readyToFire == true) {
                            timer.schedule(endFire, timeTF);
                            readyToFire = false;
                        }
                    }
                } else
                {
                    readyToFire = false;
                    mode= Mode.RESET;
                }
                break;

            case PRIME:
                //Used to specify how long to run belt for TODO: Find correct Value
                //Designed to load the next ring into the chamber - needs dev- runs after
                int beltDelay = 500;
                intakeBelt.setPower(1);

                TimerTask runBelt = new TimerTask() {
                    @Override
                    public void run() {
                        mode = Mode.RESET;
                    }
                };
                timer.schedule(runBelt, beltDelay);
                break;

            case INTAKE:
                //Moving rings up to the chamber

                //delay before intake belt is turned off, should be time that it takes ring to move
                int turnoffDelay = 4000;

                intakeBelt.setPower(1); //I do it this way so the intake can run while the robot is doing other things, then automatically turns off

                TimerTask turnoffIntake = new TimerTask() {
                    @Override
                    public void run() {
                        intakeBelt.setPower(0);
                    }
                };
                timer.schedule(turnoffIntake, turnoffDelay);
                mode = Mode.IDLE;
                break;

            case IDLE:
                //idle state, no commands being given
                break;
        }
        telemetry.addData("In firing mode", flywheel.getVelocity() * TPS_TO_RPM);
        telemetry.addData("Shooter Runtime", System.currentTimeMillis());
        telemetry.update();
        if (parentTerminated) {
            Thread.currentThread().interrupt();
            return;
        }
    }

}

