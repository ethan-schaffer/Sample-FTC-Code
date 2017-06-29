package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="RPM2Motors", group="Testing")
@Disabled
public class RPM2Motors extends OpMode
{
    DcMotor motor1, motor2, intake;
    double lastPos1 = 0;
    double deltaPos1 = 0;
    double lastPos2 = 0;
    double deltaPos2 = 0;
    double lastTime = 0;
    double deltaTime = 0;
    double targetPosition = 2200;

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
        motor1 = hardwareMap.dcMotor.get("sh1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2 = hardwareMap.dcMotor.get("sh2");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.dcMotor.get("in");
    }
    boolean lastAButtonPos = false;
    boolean lastXButtonPos = false;
    boolean lastBButtonPos = false;
    boolean lastYButtonPos = false;
    @Override
    public void loop() {
        if(motor1.getPower() == 0){
            motor1.setPower(.55);
            lastPos1 = motor1.getCurrentPosition();
            motor2.setPower(.55);
            lastPos2 = motor2.getCurrentPosition();
        }
        intake.setPower(-1); //Im lazy and don't want to reverse it

        if(gamepad1.a && !lastAButtonPos){
            targetPosition += 100;
        } else if(gamepad1.x && !lastXButtonPos){
            targetPosition -= 100;
        } else if(gamepad1.y && !lastYButtonPos){
            targetPosition += 200;
        } else if(gamepad1.b && !lastBButtonPos){
            targetPosition += 500;
        }
        lastAButtonPos = gamepad1.a;
        lastXButtonPos = gamepad1.x;
        lastBButtonPos = gamepad1.b;
        lastYButtonPos = gamepad1.y;

        deltaTime = System.currentTimeMillis() - lastTime;
        while(deltaTime < 250) {
            deltaTime = System.currentTimeMillis() - lastTime;
        }
        deltaPos1 = Math.abs(motor1.getCurrentPosition() - lastPos1);
        deltaPos2 = Math.abs(motor2.getCurrentPosition() - lastPos2);

        double RPM1 = convertTicksToRots(deltaPos1)/
                convertSecsToMins(convertMilliToSecs(deltaTime));
        double RPM2 = convertTicksToRots(deltaPos2)/
                convertSecsToMins(convertMilliToSecs(deltaTime));
/*
        if(RPM1 > RPM2 + 500){
            motor1.setPower(motor1.getPower()-.01);
            motor2.setPower(motor2.getPower());
        } else if(RPM2 > RPM1 + 500){
            motor1.setPower(motor1.getPower());
            motor2.setPower(motor2.getPower()-.01);
        } else {
            motor1.setPower(motor1.getPower());
            motor2.setPower(motor2.getPower());
        }
        if(motor1.getPower() < 1 && motor2.getPower() < 1){
            motor1.setPower(motor1.getPower()+.01);
            motor2.setPower(motor2.getPower()+.01);
        }
*/
        telemetry.addData("Target", targetPosition);
        telemetry.addData("RPM1", RPM1);
        telemetry.addData("RPM2", RPM2);
        telemetry.addData("Power1", motor1.getPower());
        telemetry.addData("Power2", motor2.getPower());
        telemetry.update();
        lastPos1 = motor1.getCurrentPosition();
        lastPos2 = motor2.getCurrentPosition();

        lastTime = System.currentTimeMillis();
    }
}
