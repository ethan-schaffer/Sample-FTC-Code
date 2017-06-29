package org.firstinspires.ftc.teamcode.Current.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R 115", group = "Red")
@Disabled
public class _Red115 extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        String LEFTPUSHNAME = "lp";//MO Port 1
        String RIGHTPUSHNAME = "rp";//MO Port 2
        Servo leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        Servo rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);
        robot.initialize(_Red115.this, hardwareMap, telemetry, true);
        while (!isStarted() && !isStopRequested()) {
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.infeed.setPower(1);
        sleep(1000);
        robot.Move(40, -1);
        robot.infeed.setPower(0);
        robot.ArcadeToAngleLeft(0, .25, -.40, 20);
        robot.AlignToWithin(.5, .05);
        robot.ShootByVoltage();
        robot.Move(55, 1.0);
        sleep(500);
        robot.EnableShot();
        robot.StopShooter();

        robot.DiagonalForwardsLeftCoast(50, 1);
        robot.SetDrivePower(0);
        sleep(100);
        while( (robot.range.getDistance(DistanceUnit.CM) > 255) && robot.range.getDistance(DistanceUnit.CM) > 50){
            robot.DiagonalForwardsLeftCoast(50, 1);
        }
        robot.DiagonalForwardsLeft(20, .75, 1);
        sleep(100);
        while( (robot.range.getDistance(DistanceUnit.CM) > 255) && robot.range.getDistance(DistanceUnit.CM) > 25){
            robot.DiagonalForwardsLeft(25, .75, .1);
        }

        robot.AlignToWithin(1.5, .05);
        robot.StrafeToWall(9, .10);

        robot.AlignToWithinOf(2, .5, .05);
        robot.FindAndPressSquareToBeacon(Robot.team.Red, .10);

        robot.CheckBeacon(Robot.team.Red);

        robot.Move(85, -1.0);
        robot.StrafeFromWall(9, .25);
        robot.AlignToWithin(3, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(-2, .5, .05);

        robot.FindAndPressSquareToBeacon(Robot.team.Red, -.10);
        robot.CheckBeacon(Robot.team.Red);

        robot.ArcadeToAngleLeft(0, .25, -.40, 7.5);
        robot.AlignToWithinOf(-17.5, 1, .05);
        robot.MoveCoast(100, - .5);
        robot.Move(25, - .25);
    }
}
