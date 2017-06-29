package org.firstinspires.ftc.teamcode.Current.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R EIAG", group = "Red")
public class RedWithLineFollow extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
/*
TODO:
Timeout on
*/
        robot.initializeWithBotton(RedWithLineFollow.this, hardwareMap, telemetry, true);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.ShootByVoltage();
        robot.Move(55, 1.0);
        robot.EnableShot(0, 1); robot.StopShooter();
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

        robot.AlignToWithin(2, .05);
        robot.Move(50, 1);
        robot.StrafeToWall(9, .10);

        robot.AlignToWithinOf(2, .5, .05);
        robot.LineSearch(2, .1);
        if(robot.colorSensorOnSide.red() < robot.colorSensorOnSide.blue()){
            robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_ON_VALUE);
        } else {
            robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_ON_VALUE);
        }
        sleep(500);
        robot.StrafeToWall(9, .10);
        robot.SetStrafePower("left", .10);
        sleep(1000);
        robot.SetDrivePower(0);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        robot.SetStrafePower("Right", .10);
        sleep(250);
        robot.SetDrivePower(0);
        robot.Move(85, - 1.0);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(2, .5, .05);
        robot.StrafeToWall(10, .10);

        robot.LineSearch(2, - .1);
        if(robot.colorSensorOnSide.red() < robot.colorSensorOnSide.blue()){
            robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_ON_VALUE);
        } else {
            robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_ON_VALUE);
        }
        sleep(500);
        robot.StrafeToWall(9, .10);
        robot.SetStrafePower("left", .10);
        sleep(500);
        robot.SetDrivePower(0);

        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);

        robot.ArcadeToAngleRight(0, .25, .50, 15);
        robot.AlignToWithinOf(135, 25, .25);
        robot.ShootByVoltage();
        robot.AlignToWithinOf(135, 3, .05);
        robot.ForwardsPLoop(145, 1.0);
        robot.AlignToWithinOf(135, 2, .05);
        robot.EnableShot();robot.StopShooter();
        robot.Move(90, 1.0);

    }
}
