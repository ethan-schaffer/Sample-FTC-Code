package org.firstinspires.ftc.teamcode.Current.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "B EIAG", group = "Blue")
public class BlueWithLineFollow extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        String LEFTPUSHNAME = "lp";//MO Port 1
        String RIGHTPUSHNAME = "rp";//MO Port 2
        Servo leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        Servo rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);
        robot.initializeWithBotton(BlueWithLineFollow.this, hardwareMap, telemetry, true);
//        robot.initialize(BlueWithLineFollow.this, hardwareMap, telemetry, true);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.MoveCoast(20, -1.0);
        robot.DiagonalBackwardsLeftCoast(80, 1);
        robot.DiagonalBackwardsLeft(25, .75, 1);
        robot.AlignToWithin(1.5, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(1, .5, .05);

        robot.LineSearch(2, .1);
        if(robot.colorSensorOnSide.red() > robot.colorSensorOnSide.blue()){
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
        robot.CheckBeacon(Robot.team.Blue);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        robot.SetStrafePower("Right", .10);
        sleep(250);
        robot.SetDrivePower(0);
        robot.Move(85, -1.0);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(-1, .5, .05);
        robot.StrafeToWall(10, .10);
        robot.LineSearch(2, -.1);
        if(robot.colorSensorOnSide.red() > robot.colorSensorOnSide.blue()){
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
        robot.CheckBeacon(Robot.team.Blue);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);

        robot.ArcadeToAngleLeft(0, .25, .40, 20);
        robot.AlignToWithinOf(-20, 1, .05);
        robot.Move(80, -.75);
    }
}
