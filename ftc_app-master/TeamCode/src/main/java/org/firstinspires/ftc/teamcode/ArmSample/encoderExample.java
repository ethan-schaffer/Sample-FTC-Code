package org.firstinspires.ftc.teamcode.ArmSample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class encoderExample extends LinearOpMode {
    DcMotor arm;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (arm.getCurrentPosition() != 0) {
            idle();
        }
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int upPosition = 1440 / 3;
        int downPosition = 0;
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                arm.setTargetPosition(upPosition);
            } else if (gamepad2.y) {
                arm.setTargetPosition(downPosition);
            }
            if (arm.isBusy()) {
                arm.setPower(1);
            } else {
                arm.setPower(0);
            }
        }
    }
}
