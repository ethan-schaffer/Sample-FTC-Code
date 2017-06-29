package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ethan Schaffer on 1/31/2017.
 */
@TeleOp(group = "TeleOp", name = "Servo Tester")
public class servoTester extends OpMode{
    Servo s;
    @Override
    public void init() {
        s = hardwareMap.servo.get("c");
        s.setPosition(0);
    }
    //open = .62 closed = 0

    @Override
    public void loop() {
        if(gamepad1.b){
            s.setPosition(s.getPosition()+.01);
        }
        else if(gamepad1.x){
            s.setPosition(s.getPosition()-.01);
        }
        telemetry.addData("S", s.getPosition());
        telemetry.update();
    }
}
