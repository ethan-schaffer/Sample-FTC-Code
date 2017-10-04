package org.firstinspires.ftc.teamcode.ArmSample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class touchSensorExample extends LinearOpMode {
    DcMotor arm;
    TouchSensor touchSensorUp, touchSensorDown;
    boolean armGoingUp = false, armGoingDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        touchSensorUp = hardwareMap.touchSensor.get("up");
        touchSensorDown = hardwareMap.touchSensor.get("down");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                //arm up!
                armGoingUp = true;
                armGoingDown = false;
            } else if (gamepad1.a) {
                //arm down!
                armGoingDown = true;
                armGoingUp = false;
            }

            if (armGoingUp && !touchSensorUp.isPressed()) {
                // if our arm should be in the up position,
                // and the touch sensor is not pressed,
                // we should set the arm power up
                arm.setPower(1);
            } else if (armGoingDown && !touchSensorDown.isPressed()) {
                // if our arm should be in the down position,
                // and the touch sensor is not pressed,
                // we should set the arm power down
                arm.setPower(-1);
            } else {
                // if neither of these conditions are true,
                // we should not set the power to anything
                arm.setPower(0);
            }
        }
    }
}
