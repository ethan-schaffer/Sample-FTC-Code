package org.firstinspires.ftc.teamcode.Current.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Blue", name = "B_FarOneBeacon")
public class BlueShootFarBeacon extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(BlueShootFarBeacon.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart();
        robot.ShootByVoltage();
        robot.Move(180, .5);
        robot.EnableShot(250, 1);
        robot.StopShooter();
        robot.AlignToWithinOf(-90, 3, .05);
        robot.Move(110, .5);
        robot.TurnRight(140, .15);
        robot.AlignToWithinOf(180, 3, .05);
        robot.Move(60, 1);
        robot.StrafeToWall(11, .10);
        robot.AlignToWithinOf(183, 1, .05);
        robot.FindAndPressBackwards(Robot.team.Blue);
        robot.StrafeFromWall(15, .20);
        robot.Move(60, - .50);
    }
}
