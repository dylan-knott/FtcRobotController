package org.firstinspires.ftc.teamcode.drive.opmode.competition;

//I'm not sure what all needs to be imported but I think I need at least the LED hardware - Greg

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.LocalizedRobotDrive;

import java.util.concurrent.TimeUnit;
import java.util.Timer;
import java.util.TimerTask;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lights {

    RevBlinkinLedDriver lights;
    RevBlinkinLedDriver.BlinkinPattern color1;
    RevBlinkinLedDriver.BlinkinPattern color2;
    RevBlinkinLedDriver.BlinkinPattern color3;
    public void initializeLights(HardwareMap hardwareMap, Telemetry telem, LocalizedRobotDrive.allianceColor clr) {

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

        if (clr.equals(LocalizedRobotDrive.allianceColor.blue))
        {
            color1 = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
            color2 = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            color3 = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
        }
        else if (clr.equals(LocalizedRobotDrive.allianceColor.red))
        {
            color1 = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            color2 = RevBlinkinLedDriver.BlinkinPattern.RED;
            color3 = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
        }

    }
}
