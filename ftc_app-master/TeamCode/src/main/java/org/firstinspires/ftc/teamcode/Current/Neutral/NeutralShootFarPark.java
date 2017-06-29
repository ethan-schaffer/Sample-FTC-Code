package org.firstinspires.ftc.teamcode.Current.Neutral;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Neutral", name = "FarPark")
public class NeutralShootFarPark extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(NeutralShootFarPark.this, hardwareMap, telemetry, false);
        while(!isStarted() && !isStopRequested()){
            robot.Housekeeping();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        sleep(1000*10);
        robot.Move(180, .5);
        robot.ShootByVoltage();
        robot.EnableShot(2000, 1.00);
        robot.StopShooter();
        robot.Move(60, .5);
        sleep(1000);
        robot.Move(30, -.5);
        robot.Move(30, .5);
    }
}
