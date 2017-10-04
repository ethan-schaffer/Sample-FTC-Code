package org.firstinspires.ftc.teamcode.TankSamples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

@TeleOp(name = "Simple Drive Tele")
public class SimpleTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get(UniversalContants.leftFrontDrive);
        DcMotor leftBack = hardwareMap.dcMotor.get(UniversalContants.leftBackDrive);
        DcMotor rightFront = hardwareMap.dcMotor.get(UniversalContants.rightFrontDrive);
        DcMotor rightBack = hardwareMap.dcMotor.get(UniversalContants.rightBackDrive);

        SixWheelDrive drive = new SixWheelDrive(SimpleTeleOp.this, leftFront, leftBack, rightFront, rightBack);
        waitForStart();
        while(opModeIsActive()) {
            gamepad1.setJoystickDeadzone(.1f);
            gamepad2.setJoystickDeadzone(.1f);
            waitForStart();

            drive.setLeft(-Math.pow(gamepad1.left_stick_y, 3));
            drive.setRight(-Math.pow(gamepad1.right_stick_y, 3));
            // Cubing the powers scales to input a bit better.
            // We can't square the input as then -1 would become +1
        }
    }

}
