package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "B 30 Defend", group = "Defend")
@Disabled
public class Blue30PointsDefend extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeWithBotton(Blue30PointsDefend.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.Housekeeping();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.MoveCoast(15, -1.0);
        robot.DiagonalBackwardsLeftCoast(25, 1);
        robot.AlignToWithin(3, .05);
        robot.StrafeToWall(12, .10);
        robot.AlignToWithin(1, .05);
        robot.LineSearch(2, - .20);
        sleep(250);
        robot.LineSearch(2, .05);
        robot.StrafeToWall(9, .10);
        robot.PressBeacon(Robot.team.Blue);

        robot.StrafeFromWall(11, .50);

        robot.TurnLeftEnc(35, .45);
        robot.AlignToWithinOf(90, 1, .05);
        robot.MoveCoast(110, - 1.0);
        robot.MoveCoast(25, 0.75);
        robot.MoveCoast(25, - 0.60);
        robot.MoveCoast(25, 0.50);
    }
}
