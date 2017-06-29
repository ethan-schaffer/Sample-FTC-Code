package org.firstinspires.ftc.teamcode.Current.Defensive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Defend", name = "Blue_Defend")
public class New_DefendB extends LinearOpMode {
    Robot robot = new Robot();
    boolean Forwards = true;
    boolean lastA = false;
    boolean lastL = false;
    boolean lastU = false;
    boolean lastD = false;
    boolean lastR = false;
    double sleepTime;
    boolean lastY = false;
    double angTarg = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(New_DefendB.this, hardwareMap, telemetry, true);
        angTarg = -25;
        while (!isStarted() && !isStopRequested()) {
            if(gamepad1.dpad_left && !lastL){
                angTarg-=.25;
            } else if(gamepad1.dpad_right && !lastR){
                angTarg+=.25;
            } else if(gamepad1.dpad_up && !lastU){
                angTarg+=1;
            } else if(gamepad1.dpad_down && !lastD){
                angTarg-=1;
            }
            if(gamepad1.a && !lastA){
                sleepTime = Range.clip(sleepTime + .25, 0, 15);
            } else if(gamepad1.y && !lastY){
                sleepTime = Range.clip(sleepTime - .25, 0, 15);
            }
            lastL = gamepad1.dpad_left;
            lastR = gamepad1.dpad_right;
            lastA = gamepad1.a;
            lastU = gamepad1.dpad_up;
            lastD = gamepad1.dpad_down;
            lastY = gamepad1.y;
            telemetry.addData("Sleep Time", sleepTime);
            telemetry.addData("Angle At End", angTarg);
            telemetry.addData("Yaw", robot.navX.getYaw());
            telemetry.update();
        }
        waitForStart();
        double stTime = getRuntime();
        sleep((long)(sleepTime*1000));
        robot.ShootByVoltage();
        robot.ForwardsPLoop(120, 1);
        sleep(100);
        robot.AlignToWithinOf(45, 5, .15);
        robot.AlignToWithinOf(45, 1, .05);
        robot.EnableShot(250, 1);
        robot.StopShooter();

        robot.Move(130, 1);
        robot.AlignToWithin(1, .05);
        while ((getRuntime() - stTime) < 10) {
            sleep(1);
        }
        robot.Move(115, 1);
        robot.AlignToWithinOf(angTarg, 1.5, .05);
        robot.Move(100, 1);
        telemetry.clear();
        telemetry.addData("Time", getRuntime()-stTime);
        telemetry.update();
        while(getRuntime()-stTime<23){
            sleep(1);
        }
        robot.AlignToWithin(2, .05);
        robot.Move(150, -1);
    }
}
