package org.firstinspires.ftc.teamcode.Current.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R 95", group = "Red")
public class _Red95New extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
/*
TODO:
Timeout on
*/
        robot.initialize(_Red95New.this, hardwareMap, telemetry, true);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.ShootByVoltage();
        robot.Move(55, 1.0);
        sleep(250);
        robot.EnableShot();
        robot.StopShooter();
        robot.SetStrafePower("Left", .5);
        sleep(250);
        robot.DiagonalForwardsLeftCoast(50, 1);
        robot.SetDrivePower(0);
        sleep(50);
        while( (robot.range.getDistance(DistanceUnit.CM) > 255) && robot.range.getDistance(DistanceUnit.CM) > 50){
            robot.DiagonalForwardsLeft(50, 1);
        }
        robot.DiagonalForwardsLeft(20, .75, 1);
        sleep(50);
        while( (robot.range.getDistance(DistanceUnit.CM) > 255) && robot.range.getDistance(DistanceUnit.CM) > 25){
            robot.DiagonalForwardsLeft(25, .75, .1);
        }

        robot.AlignToWithin(1.5, .05);
        robot.StrafeToWall(9, .10);

        robot.AlignToWithinOf(2, .5, .05);
        int threshold = 2;
        if(robot.colorSensorOnSide.red() > threshold || robot.colorSensorOnSide.blue() > threshold){
            while(robot.colorSensorOnSide.red() > threshold || robot.colorSensorOnSide.blue() > threshold){
                robot.SetDrivePower(-.15);
            }
            sleep(100);
            robot.SetDrivePower(0);
        }

        robot.FindAndPressSquareToBeacon(Robot.team.Red, .12);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        robot.CheckBeacon(Robot.team.Red);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        robot.SetStrafePower("Right", .10);
        sleep(250);
        robot.SetDrivePower(0);
        robot.Move(85, -1.0);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(-2, .5, .05);
        robot.StrafeToWall(10, .10);
        robot.FindAndPressSquareToBeacon(Robot.team.Red, -.12);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        robot.CheckBeacon(Robot.team.Red);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);

        robot.ArcadeToAngleLeft(0, .25, -.50, 7.5);
        robot.AlignToWithinOf(-17.5, 4, .10);
        robot.MoveCoast(85, - .5);
        robot.Move(25, - .25);
    }
}
