package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Ethan Schaffer on 9/24/2016.
 */
@TeleOp(name="CR Servo", group = "TeleOpOld")
@Disabled
public class CRinfeed extends OpMode{
    CRServo left, right;

    @Override
    public void init() {
        left = hardwareMap.get(CRServo.class, "l");
        right = hardwareMap.get(CRServo.class, "r");
    }

    @Override
    public void loop() {
        left.setPower(gamepad1.left_stick_y);
        right.setPower(-gamepad1.left_stick_y);
    }
}
