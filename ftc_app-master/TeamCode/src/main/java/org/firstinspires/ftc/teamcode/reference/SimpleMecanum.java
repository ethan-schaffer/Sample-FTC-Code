package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

/**
 * Created by Ethan Schaffer on 11/11/2016.
 */

@TeleOp(name="Tele Op Mecanum ", group = "TeleOpOld")
@Disabled
public class SimpleMecanum extends OpMode {
    DcMotor leftFront, leftBack, rightBack, rightFront;
    public boolean ConditionToReverseMotors(){
        return gamepad1.a;
    }

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront = hardwareMap.dcMotor.get("right_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
    }

    @Override
    public void loop() {
        if (ConditionToReverseMotors()){
            leftFront.setDirection( leftFront.getDirection() == DcMotorSimple.Direction.REVERSE ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection( leftBack.getDirection() == DcMotorSimple.Direction.REVERSE ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection( rightFront.getDirection() == DcMotorSimple.Direction.REVERSE ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection( rightBack.getDirection() == DcMotorSimple.Direction.REVERSE ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
            //Ternary Operations
        }

        arcadeMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, leftFront, rightFront, leftBack, rightBack);
    }
    // y - forwards
    // x - side
    // c - rotation
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
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
