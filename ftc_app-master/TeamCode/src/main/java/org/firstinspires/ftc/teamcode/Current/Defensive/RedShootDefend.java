package org.firstinspires.ftc.teamcode.Current.Defensive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Defend", name = "R_Defend")
@Disabled
public class RedShootDefend extends LinearOpMode {
    Robot robot = new Robot();
    boolean lastY = false;
    boolean lastA = false;
    double sleepTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(RedShootDefend.this, hardwareMap, telemetry, true);
        while (!isStarted() && !isStopRequested()) {
            if(gamepad1.a && !lastA){
                sleepTime = Range.clip(sleepTime + .25, 0, 15);
            } else if(gamepad1.y && !lastY){
                sleepTime = Range.clip(sleepTime - .25, 0, 15);
            }
            lastA = gamepad1.a;
            lastY = gamepad1.y;
            telemetry.addData("Sleep Time", sleepTime);
            telemetry.update();
            robot.sensorsInfo();
        }
        waitForStart();
        double stTime = getRuntime();
        sleep((long)(sleepTime*1000));
        robot.ShootByVoltage();
        robot.ForwardsPLoop(180, 1);
        robot.EnableShot(250, 1);
        robot.StopShooter();
        robot.Move(115, 1);
        robot.AlignToWithin(5, .15);
        robot.StrafeLeft(15, 1.0);
        robot.AlignToWithin(1.5, .05);
        while ((getRuntime() - stTime) < 9.750) {
            sleep(1);
        }
        robot.Move(215, 1);
        sleep(4500);
    }
}
