package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ethan Schaffer on 11/5/2016.
 */
@TeleOp(name="Linear Servo", group="TeleOpOld")
@Disabled
public class LinearServo extends OpMode {
    Servo linearL, linearR;

    @Override
    public void init() {

        linearL = hardwareMap.servo.get("lp");
        linearR = hardwareMap.servo.get("rp");

    }

    @Override
    public void loop() {
        double valueL = gamepad1.left_stick_y/2+.5;
        linearL.setPosition(valueL);
        telemetry.addData("L", linearL.getPosition());

        double valueR = gamepad1.right_stick_y/2+.5;
        linearR.setPosition(valueR);
        telemetry.addData("R", linearR.getPosition());
        telemetry.update();
    }
}
