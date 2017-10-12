package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class SampleControllerUser extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Controllers controllers = new Controllers();
        controllers.update(this);
        while(opModeIsActive()){
            if(!controllers.a && gamepad1.a){
                telemetry.addLine("a is being pressed or held down");
            }
            if(controllers.a && !gamepad1.a){
                telemetry.addLine("a was just released!");
            }
            telemetry.update();
            controllers.update(this);
        }
    }

}
