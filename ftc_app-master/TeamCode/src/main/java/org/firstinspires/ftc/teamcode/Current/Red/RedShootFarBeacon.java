package org.firstinspires.ftc.teamcode.Current.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Red", name = "R_FarOneBeacon")
public class RedShootFarBeacon extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(RedShootFarBeacon.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart();
        robot.ShootByVoltage();
        robot.Move(180, .5);
        robot.EnableShot(250, 1);
        robot.StopShooter();
        robot.AlignToWithinOf(90, 3, .05);
        robot.Move(110, .5);
        robot.AlignToWithinOf(0, 3, .05);
        robot.Move(60, -.5);
        robot.StrafeToWall(10, .10);
        robot.FindAndPressForwards(Robot.team.Red);
        robot.StrafeFromWall(15, .20);
        robot.Move(60, .50);
    }
}
