package org.firstinspires.ftc.teamcode.Current.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Red", name = "R_FarParkRamp")
public class RedShootFarRamp extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(RedShootFarRamp.this, hardwareMap, telemetry, true);
        waitForStart();
        sleep(1000*5);
        robot.ShootByVoltage();
        robot.Move(180, .5);
        robot.EnableShot(250, 1);
        sleep(100);
        robot.EnableShot(1000, 1);
        robot.StopShooter();
        robot.TurnLeft(45, .05);
        robot.Move(90, .5);
        robot.TurnLeftEnc(20, .50);
        robot.Move(110, .5);
    }
}
