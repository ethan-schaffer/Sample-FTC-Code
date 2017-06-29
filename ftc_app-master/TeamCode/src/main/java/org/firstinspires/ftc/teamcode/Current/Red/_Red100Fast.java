package org.firstinspires.ftc.teamcode.Current.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R Fast", group = "Red")
@Disabled
public class _Red100Fast extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        String LEFTPUSHNAME = "lp";//MO Port 1
        String RIGHTPUSHNAME = "rp";//MO Port 2
        Servo leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        Servo rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);
        robot.initialize(_Red100Fast.this, hardwareMap, telemetry, true);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.MoveCoast(50, 1.0);
        robot.ToRangeByArcade(10, 1, 0, .2);
        robot.AlignToWithin(1.5, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(2, .5, .05);

        int threshold = 2;
        if(robot.colorSensorOnSide.red() > threshold || robot.colorSensorOnSide.blue() > threshold){
            while(robot.colorSensorOnSide.red() > threshold || robot.colorSensorOnSide.blue() > threshold){
                robot.SetDrivePower(-.15);
            }
            sleep(100);
            robot.SetDrivePower(0);
        }

        robot.FindAndPressSquareToBeacon(Robot.team.Blue, .12);
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
        robot.AlignToWithinOf(-2, .5, .05);
        robot.StrafeToWall(10, .10);
        robot.FindAndPressSquareToBeacon(Robot.team.Blue, -.12);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);
        robot.CheckBeacon(Robot.team.Blue);
        robot.leftButtonPusher.setPosition(robot.LEFT_SERVO_OFF_VALUE);
        robot.rightButtonPusher.setPosition(robot.RIGHT_SERVO_OFF_VALUE);

        robot.ShootByVoltage();
        robot.ArcadeToAngleRight(0, .25, .40, 20);
        robot.AlignToWithinOf(90, 1, .05);
        robot.ForwardsPLoop(85, 1.0);
        robot.AlignToWithinOf(90, 1, .05);
        robot.EnableShot();robot.StopShooter();
        robot.AlignToWithinOf(45, 1, .05);
        robot.Move(70, -1.0);
    }
}
