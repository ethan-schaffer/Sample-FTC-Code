package org.firstinspires.ftc.teamcode.Blue;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Blue", name = "B_2B (Defend)")
@Disabled
public class Blue2BDefend extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeWithBotton(Blue2BDefend.this, hardwareMap, telemetry, true);
        waitForStart();
        robot.Move(40, - 1.00);
        robot.TurnRight(35, 0.15);
        robot.Move(240, - 1.00);
        robot.AlignToWithin(3, 0.05);
        //Line up with the wall
        robot.StrafeToWall(15, 0.10);

        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2, 0.10);
        robot.LineSearch(2, - 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(1.0, 0.04);
        robot.StrafeToWall(9, 0.10);

        robot.Move(2, - 1.00);
        robot.LineSearch(2, 0.05);
        robot.PressBeacon(Robot.team.Blue );
        //Press the first beacon

        robot.StrafeFromWall(15, 1.0);
        robot.AlignToWithin(3, 0.05);
        robot.Move(140, - 1.00);
        robot.LineSearch(2, - 0.11);
        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.StrafeToWall(10, 0.10);
        robot.Move(1, 1.00);
        robot.LineSearch(2, - 0.10);
        robot.LineSearch(2, 0.05);
        robot.AlignToWithin(5, 0.05);
        robot.StrafeToWall(8, 0.10);
        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(1, 0.04);
        robot.PressBeacon(Robot.team.Blue);
        //Press the second beacon

        robot.StrafeFromWall(13, 1.00);
        robot.TurnLeftEnc(45, .50);
        robot.Move(15, -1.0);
    }
}
