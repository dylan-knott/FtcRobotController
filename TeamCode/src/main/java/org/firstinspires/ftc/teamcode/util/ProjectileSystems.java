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
    private Map<Float, Integer> distToRPM = new HashMap<>();

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

        deflector.setPosition(0);
        indexer.setPosition(0);
        reloader.setPosition(0);

        mode = Mode.IDLE;
    }

    /*******************************************UNUSED*******************************************/

    public void fireRing(float dist)
    {
        //TODO: connection of distance to deflector angle
        setDeflector(90);
        mode = ProjectileSystems.Mode.FIRING;
    }

    public void setFlywheel(float inputPower)
    {
        //Remap input to the max power
        float power = (float) (inputPower * flywheelPower);

        flywheel.setPower(power);
    }

    public void setDeflector(double position){
        deflector.setPosition(position / 270.0f);
    }

    public void setFlywheelRPM(double rpm)
    {

        double flywheelAngularVelocity = RPM_TO_TPS * rpm;
        flywheel.setVelocity(flywheelAngularVelocity);
        telemetry.addData("Flywheel RPM: ", flywheelAngularVelocity);

    }
    public void setIndexer(double pos){
        indexer.setPosition(pos / 200.0f);
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

            case CHAMBER:
                //TODO: TEST IF THIS WORKS, MAY NOT COMPLETE. FIGURE OUT HOW TO OPEN, THEN WAIT FOR CLOSE TO START INTAKE MAY BE IDIOT
                //Used to specify how long to wait to close chambering servo TODO: Find correct value
                int reloaderDelay = 500;

                //needs testing
                reloader.setPosition(reloadPOS);
                reloader.setPosition(0);

                //after
                TimerTask servoClose = new TimerTask() {
                    @Override
                    public void run() {
                        mode = Mode.PRIME;
                    }
                };
                timer.schedule(servoClose, reloaderDelay);
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
                //Desnigned to load the next ring into the chamber - needs dev- runs after CHAMBER
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
                intakeBelt.setPower(1);
                mode = Mode.IDLE;
                break;

            case IDLE:
                //idle state, no commands being given
                break;
        }
    }

}

