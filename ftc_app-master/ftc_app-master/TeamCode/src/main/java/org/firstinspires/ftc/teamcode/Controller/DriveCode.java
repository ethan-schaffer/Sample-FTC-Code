package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

@TeleOp(name = "Simple Controller")
public class DriveCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Controllers controllers = new Controllers();
        controllers.update(this);
        while(opModeIsActive()){
            if(!controllers.gamepad1.a && gamepad1.a){
                arm.setPower(1);
            } else {
                arm.setPower(0);
            }
            controllers.update(this);
        }
    }

}
