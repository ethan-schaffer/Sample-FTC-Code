package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R Do It All", group = "Main")
@Disabled
public class aaaaRed100Points extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeWithBotton(aaaaRed100Points.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.Housekeeping();
        }
        waitForStart();
        robot.ShootByVoltage();
        robot.Move(65, 1.0);
        robot.EnableShot(250, 1);
        robot.StopShooter();
        robot.Move(10, - 0.75);
        boolean didCrash = robot.DiagonalForwardsLeft(45, 1);
        robot.StrafeToWall(25, .5);
        robot.StrafeFromWall(15, .25);
        sleep(100);
        robot.AlignToWithin(.5, .05);
        if(!didCrash) {
            robot.LineSearch(2, .15);
            sleep(100);
            robot.LineSearch(2, -.05);
        } else {
            robot.LineSearch(2, -.15);
            sleep(100);
            robot.LineSearch(2, .05);
        }
        robot.StrafeToWall(10, .08);
        robot.StrafeToWall(9, .07);
        robot.PressBeacon(Robot.team.Red);

        robot.StrafeFromWall(13, 1.0);
        robot.AlignToWithin(1, .05);
        if (!didCrash){
            robot.Move(130, -1.0);
        } else {
            robot.Move(130, 1.0);
        }

        robot.AlignToWithin(1, .05);

        if(!didCrash){
            robot.LineSearch(2, -.15);
        } else {
            robot.LineSearch(2, .15);
        }
        sleep(100);
        robot.StrafeToWall(10, .08);
        robot.StrafeToWall(9, .07);
        robot.AlignToWithin(1, .05);
        if(!didCrash){
            robot.LineSearch(2, .05);
        } else {
            robot.LineSearch(2, -.05);
        }
        robot.StrafeToWall(9, .07);
        robot.PressBeacon(Robot.team.Red);
        robot.StrafeFromWall(25, 1.0);
        if(!didCrash){
            robot.AlignToWithin(1.5, .05);
            robot.Move(115, -1.0);
            robot.Move(25, -0.25);
        } else {
            robot.TurnRightEnc(35, 0.50);
            robot.Move(15, 1.0);
            robot.Move(15, -.75);
            robot.Move(15, 0.50);
            robot.Move(15, -0.25);
        }

    }
}
