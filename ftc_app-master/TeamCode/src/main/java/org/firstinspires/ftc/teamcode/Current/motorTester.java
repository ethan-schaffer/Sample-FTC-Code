package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ethan Schaffer on 3/15/2017.
 */

@TeleOp(name = "Motor Tester", group = "TeleOp")
public class motorTester extends OpMode {
    DcMotor leftF, rightF, leftB, rightB;
    @Override
    public void init() {
        leftF = hardwareMap.dcMotor.get("l1");
        leftB = hardwareMap.dcMotor.get("l2");
        rightF = hardwareMap.dcMotor.get("r1");
        rightB = hardwareMap.dcMotor.get("r2");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            leftF.setPower(1);
        } else {
            leftF.setPower(0);
        }
        if(gamepad1.b){
            leftB.setPower(1);
        } else {
            leftB.setPower(0);
        }
        if(gamepad1.x){
            rightF.setPower(1);
        } else {
            rightF.setPower(0);
        }
        if(gamepad1.y){
            rightB.setPower(1);
        } else {
            rightB.setPower(0);
        }
    }
}
