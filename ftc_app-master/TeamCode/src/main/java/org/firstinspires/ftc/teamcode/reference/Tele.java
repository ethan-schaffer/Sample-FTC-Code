package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

/**
 * Created by Ethan Schaffer on 9/24/2016.
 */
@TeleOp(name="Test: Tele", group = "TeleOpOld")
@Disabled
public class Tele extends OpMode{
    DcMotor left, right;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "l");
        right = hardwareMap.get(DcMotor.class, "r");
    }

    @Override
    public void loop() {
        left.setPower(gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);
    }
}
