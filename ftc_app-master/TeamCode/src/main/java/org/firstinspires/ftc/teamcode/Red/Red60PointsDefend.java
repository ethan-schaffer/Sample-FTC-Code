package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R 60 Defend", group = "Defend")
@Disabled
public class Red60PointsDefend extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeWithBotton(Red60PointsDefend.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.Housekeeping();
        }
        waitForStart();
        robot.ShootByVoltage();
        robot.Move(65, 1.0);
        robot.EnableShot(250, 1);
        robot.StopShooter();
        robot.DiagonalForwardsLeft(25, 1);
        sleep(100);
        robot.AlignToWithin(.5, .05);
        robot.LineSearch(2, .15);
        sleep(100);
        robot.LineSearch(2, -.05);
        robot.StrafeToWall(10, .08);
        robot.StrafeToWall(9, .07);
        robot.PressBeacon(Robot.team.Red);
        robot.TurnRightEnc(35, 0.50);
        robot.Move(15, 1.0);
        robot.Move(15, -.75);
        robot.Move(15, 0.50);
        robot.Move(15, -0.25);
    }
}
