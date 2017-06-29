package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="RPM1Motor", group="Testing")
@Disabled
public class RPM1Motor extends OpMode
{
    DcMotor motor1;
    double lastPos = 0;
    double deltaPos = 0;
    double lastTime = 0;
    double deltaTime = 0;
    double targetRPM = 1900;

    double convertTicksToRots(double ticks){
        return ticks/(28*4);
    }
    double convertMilliToSecs(double milli){
        return milli/1000;
    }
    double convertSecsToMins(double seconds){
        return seconds/60;
    }

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("s1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    boolean lastAButtonPos = false;
    boolean lastXButtonPos = false;
    boolean lastBButtonPos = false;
    boolean lastYButtonPos = false;
    @Override
    public void loop() {
        if(motor1.getPower() == 0){
            motor1.setPower(.5);
        }
        if(gamepad1.a && !lastAButtonPos){
            targetRPM += 100;
        } else if(gamepad1.x && !lastXButtonPos){
            targetRPM -= 100;
        } else if(gamepad1.y && !lastYButtonPos){
            targetRPM += 200;
        } else if(gamepad1.b && !lastBButtonPos){
            targetRPM += 500;
        }
        lastAButtonPos = gamepad1.a;
        lastXButtonPos = gamepad1.x;
        lastBButtonPos = gamepad1.b;
        lastYButtonPos = gamepad1.y;
        deltaPos = Math.abs((motor1.getCurrentPosition() - lastPos));
        deltaTime = System.currentTimeMillis() - lastTime;
        double RPM = convertTicksToRots(deltaPos)/
                convertSecsToMins(convertMilliToSecs(deltaTime));
        if(targetRPM > RPM){
            motor1.setPower(motor1.getPower() + .01);
        }
        if(targetRPM < RPM){
            motor1.setPower(motor1.getPower() - .01);
        }

        telemetry.addData("Target", targetRPM);
        telemetry.addData("RPM", RPM);
        telemetry.addData("Power", motor1.getPower());
        telemetry.update();
        lastPos = (motor1.getCurrentPosition());
        lastTime = System.currentTimeMillis();
    }
}
