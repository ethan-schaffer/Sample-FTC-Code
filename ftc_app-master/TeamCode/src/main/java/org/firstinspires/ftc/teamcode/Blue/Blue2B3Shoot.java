package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Blue", name = "B_2B (Shoot 3)")
@Disabled
public class Blue2B3Shoot extends LinearOpMode{
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeWithBotton(Blue2B3Shoot.this, hardwareMap, telemetry, true);
        waitForStart();

        robot.infeed.setPower(1);
        sleep(500);
        robot.infeed.setPower(0);
        robot.TurnRight(90, .35);
        robot.AlignToWithinOf(2.5, - 90, .15);
        robot.infeed.setPower(0);
        robot.Move(70, - 1.00);
        robot.TurnRight(55, 0.15);
        robot.Move(240, - 1.00);
        robot.AlignToWithinOf(3, - 90, .05);
        //Line up with the wall
        robot.StrafeToWall(15, 0.10);

        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.LineSearch(2, - 0.10);
        robot.LineSearch(2,   0.05);
        robot.StrafeToWall(9, 0.10);

        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);
        robot.LineSearch(2,   0.05);
        robot.PressBeacon(Robot.team.Blue );
        //Press the first beacon
        robot.StrafeFromWall(15, 1.0);
        robot.AlignToWithinOf(2.5, 90, .05);
        robot.AlignToWithinOf(2.5, 90, .05);
        robot.Move(140, 1.00);
        robot.AlignToWithinOf(2.5, 90, .05);
        robot.LineSearch(2, 0.11);
        robot.AlignToWithinOf(2.5, 90, .05);
        robot.StrafeToWall(9, 0.10);
        robot.AlignToWithinOf(2.5, 90, .05);
        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);
        robot.PressBeacon(Robot.team.Blue);
        //Press the second beacon

        robot.StrafeFromWall(13, 1.00);
        robot.ShootByVoltage();
//        robot.ShootAtPower(0, 0.80); //Turn on the shooter so it can speed up without wasting time
        robot.TurnRight(35, 0.10);
        robot.Move(145, 1.00);
        sleep(250);
        robot.EnableShot(750, 1.00);
        //Shoot the particles into the vortex
        robot.StopShooter();
//        robot.ShootAtPower(0, 0.00);
        robot.Move(60, 1.00);
        //Go park on the wooden part of the field
    }
}
