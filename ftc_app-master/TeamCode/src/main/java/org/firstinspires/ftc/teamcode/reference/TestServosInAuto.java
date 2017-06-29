package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Current.*;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "Servo", group = "New")
@Disabled
public class TestServosInAuto extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Current.Robot robot = new org.firstinspires.ftc.teamcode.Current.Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        String LEFTPUSHNAME = "lp";//MO Port 1
        String RIGHTPUSHNAME = "rp";//MO Port 2
        Servo leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        Servo rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        leftButtonPusher.setPosition(1);
        rightButtonPusher.setPosition(1);
        sleep(5000);
        leftButtonPusher.setPosition(.35);
        rightButtonPusher.setPosition(.35);
        sleep(5000);

    }
}
