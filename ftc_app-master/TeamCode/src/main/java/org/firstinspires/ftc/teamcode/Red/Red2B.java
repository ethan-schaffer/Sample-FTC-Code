package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Red", name = "R_2B")
@Disabled
public class Red2B extends LinearOpMode{
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeWithBotton(Red2B.this, hardwareMap, telemetry, true);
        waitForStart();

        robot.ShootByVoltage();
        robot.Move(80, 1.00);
        robot.EnableShot(850, 1.00);
        robot.infeed.setPower(0);
        robot.StopShooter();
        robot.TurnLeft(35, 0.15);
        robot.Move(240, 1.00);
        robot.AlignToWithin(3, 0.05);
        //Line up with the wall
        robot.StrafeToWall(15, 0.10);

        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2, - 0.10);
        robot.LineSearch(2,   0.05);
        robot.StrafeToWall(9, 0.13);
        robot.StrafeToWall(8, 0.05);

        robot.LineSearch(2,   0.10);

        robot.PressBeacon(Robot.team.Red );
        //Press the first beacon
        robot.StrafeFromWall(15, 1.0);
        robot.AlignToWithin(2, 0.05);
        robot.Move(130, 1.00);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2, 0.11);
        robot.AlignToWithin(2.5, 0.05);
        robot.StrafeToWall(9, 0.10);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);
        robot.PressBeacon(Robot.team.Red);
        //Press the second beacon

        robot.StrafeFromWall(13, 1.00);
        robot.TurnLeftEnc(35, 1.00);
        robot.Move(215, -1.00);
    }
}
