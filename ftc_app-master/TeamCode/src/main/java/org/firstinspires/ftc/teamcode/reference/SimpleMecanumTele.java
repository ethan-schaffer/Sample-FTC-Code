package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@TeleOp(name="Mecanum", group="TeleOpOld")
@Disabled
public class SimpleMecanumTele extends OpMode {
    DcMotor left1, left2, right1, right2;

    @Override
    public void init() {
        left1 = hardwareMap.dcMotor.get("l1");
        left2 = hardwareMap.dcMotor.get("l2");
        right1 = hardwareMap.dcMotor.get("r1");
        right2 = hardwareMap.dcMotor.get("r2");
        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        arcade(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, left1, right1, left2, right2);
    }

    //Mecanum:
    // y - forwards
    // x - side
    // c - rotation
    public static void arcade(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }
}