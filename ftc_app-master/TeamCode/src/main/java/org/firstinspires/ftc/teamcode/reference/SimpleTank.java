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

@TeleOp(name="Tele Op Tank", group = "TeleOpOld")
@Disabled
public class SimpleTank extends OpMode {
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
        leftFront.setPower(gamepad1.left_stick_y);
        leftBack.setPower(gamepad1.left_stick_y);
        rightFront.setPower(gamepad1.right_stick_y);
        rightBack.setPower(gamepad1.right_stick_y);
    }
}
