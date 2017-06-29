package org.firstinspires.ftc.teamcode.Current.Defensive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Defend", name = "B_Defend")
@Disabled
public class BlueShootDefend2 extends LinearOpMode {
    Robot robot = new Robot();
    boolean Forwards = true;
    boolean lastA = false;
    double sleepTime;
    boolean lastY = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(BlueShootDefend2.this, hardwareMap, telemetry, true);
        while (!isStarted() && !isStopRequested()) {
            if(gamepad1.a && !lastA){
                sleepTime = Range.clip(sleepTime + .25, 0, 15);
            } else if(gamepad1.y && !lastY){
                sleepTime = Range.clip(sleepTime - .25, 0, 15);
            }
            lastA = gamepad1.a;
            lastY = gamepad1.y;
            telemetry.addData("Sleep Time", sleepTime);
            telemetry.addData("Yaw", robot.navX.getYaw());
            telemetry.update();
        }
        waitForStart();
        double stTime = getRuntime();
        sleep((long)(sleepTime*1000));
        robot.ShootByVoltage();
        robot.ForwardsPLoop(120, 1);
        sleep(100);
        robot.AlignToWithinOf(45, 5, .10);
        robot.AlignToWithinOf(45, 1, .05);
        robot.EnableShot(250, 1);
        robot.StopShooter();

        robot.Move(130, 1);
        robot.AlignToWithin(1, .05);
        while ((getRuntime() - stTime) < 10.5) {
            sleep(1);
        }
        robot.Move(115, 1);
        robot.TurnLeft(5, .05);
        robot.Move(100, 1);
        telemetry.clear();
        telemetry.addData("Time", getRuntime()-stTime);
        telemetry.update();
        while(opModeIsActive()){
            sleep(1);
        }
    }
}
