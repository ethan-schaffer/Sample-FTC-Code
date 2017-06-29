package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Red", name = "R_S2B (No Park)")
@Disabled

public class Red2BSNoPark extends LinearOpMode{
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeWithBotton(Red2BSNoPark.this, hardwareMap, telemetry, true);
        waitForStart();

        robot.ShootByVoltage();
        robot.Move(80, 1.00);
        robot.EnableShot(750, 1.00);
        robot.infeed.setPower(0);
        robot.ShootByVoltage();
        robot.TurnLeft(35, 0.15);
        robot.Move(240, 1.00);
        robot.AlignToWithin(3, 0.05);
        //Line up with the wall
        robot.StrafeToWall(15, 0.10);

        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2, - 0.10);
        robot.LineSearch(2,   0.05);
        robot.StrafeToWall(9, 0.10);

        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);

        robot.PressBeacon(Robot.team.Red );
        //Press the first beacon
        robot.StrafeFromWall(15, 1.0);
        robot.AlignToWithin(2, 0.05);
        robot.Move(140, 1.00);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2, 0.11);
        robot.AlignToWithin(2.5, 0.05);
        robot.StrafeToWall(9, 0.10);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);
        robot.PressBeacon(Robot.team.Red);
        //Press the second beacon

        robot.StrafeFromWall(20, 1.00);
    }
}
