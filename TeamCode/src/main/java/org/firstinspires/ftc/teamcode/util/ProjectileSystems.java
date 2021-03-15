package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;

import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

public class ProjectileSystems
{
    //Hardware declaration, need color sensors
    public DcMotor intakeBelt;
    public DcMotorEx flywheel;
    public Servo indexer, reloader, deflector;


    public double flywheelPower = 1;
    public double intakePower = 1;

    Telemetry telemetry = null;
    LocalizedRobotDrive.allianceColor teamColor = null;

    private final double RPM_TO_TPS = 28.0f /60;
    private final double TPS_TO_RPM = 60/28.0f;
    private Map<Float, Integer> distToAngle4500  = new HashMap<>();;


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
        mode = Mode.IDLE;
    }

    /*******************************************UNUSED*******************************************/

    public void fireRing(float dist)
    {
        //TODO: connection of distance to deflector angle
        setDeflector(90);
        mode = ProjectileSystems.Mode.FIRING;
    }

    public void setDeflector(double position){
        deflector.setPosition(position / 170.0f);
    }

    public void setFlywheelRPM(double rpm)
    {

        double flywheelAngularVelocity = RPM_TO_TPS * rpm;
        flywheel.setVelocity(flywheelAngularVelocity);
        telemetry.addData("Flywheel RPM: ", flywheelAngularVelocity);

    }

    //Returns if the shooter is Idle or busy
    public boolean isBusy() {
        if (mode == Mode.IDLE) return false;
        else return true;
    }

    public void update()
    {
        //placeholder, need to figure out once built
        double firePOS = 180;
        double reloadPOS = 180;

        switch (mode) {
            case RESET:
                //set all moving system to default position/off
                setFlywheelRPM(0);
                indexer.setPosition(0);
                intakeBelt.setPower(0);//Set Velo instead?
                reloader.setPosition(0);
                deflector.setPosition(0);
                mode = Mode.IDLE;
                break;

            case FIRING:
                //Fire the current ring, and set mode to reset

                //time alloted for spinup before firing-may be remove
                int timeTF = 500;
                double flywheelRPM = 4500;
                //Turn on flywheel to set RPM -Find correct rpm, make it changeable?
                setFlywheelRPM(flywheelRPM);

                telemetry.addData("In firing mode", flywheel.getVelocity() * TPS_TO_RPM);

                //Mix of new untested code(Get velo statement) and old(setting flywheel power to full, wait timeTF, then move indexer, and reset
                if (flywheel.getVelocity() * TPS_TO_RPM <= flywheelRPM + 20 && flywheel.getVelocity() * TPS_TO_RPM >= flywheelRPM - 20 )
                {
                    indexer.setPosition(50.0/280.0f);

                    TimerTask endFire = new TimerTask() {
                        @Override
                        public void run() {
                            mode = Mode.RESET;
                        }
                    };

                    timer.schedule(endFire, timeTF);

                }
                break;

            case PRIME:
                //Used to specify how long to run belt for TODO: Find correct Value
                //Desnigned to load the next ring into the chamber - needs dev- runs after
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
    }

}

