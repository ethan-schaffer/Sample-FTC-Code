package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Defend", name = "B_Far1Beacon")
@Disabled
public class Blue1BeaconFar extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeWithBotton(Blue1BeaconFar.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.Housekeeping();
        }
        waitForStart();
        robot.ShootByVoltage();
        robot.Move(170, 1);
        robot.EnableShot(250, 1);
        robot.StopShooter();
        robot.Move(20, -1);
        robot.TurnRight(90, .05);
        robot.Move(100, .5);
        robot.AlignToWithinOf(45, 1.5, .05);
        robot.Move(180, 1.0);
        robot.TurnRight(150, .35);
        robot.AlignToWithinOf(180, 1.5, .05);
        robot.StrafeToWall(15, .10);
        robot.LineSearch(2, .20);
        sleep(250);
        robot.LineSearch(2, - .05);
        robot.StrafeToWall(9, .10);
        robot.PressBeacon(Robot.team.Blue);
        robot.StrafeFromWall(25, .75);
        robot.AlignToWithinOf(180, 1.5, .05);
        robot.Move(115, 1.0);
        robot.Move(25, 0.25);

    }
}
