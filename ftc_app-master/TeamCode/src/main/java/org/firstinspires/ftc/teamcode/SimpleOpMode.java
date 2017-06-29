package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Simple Drive Program", group="Tele Op")  // @Autonomous(...) is the other common choice
// we declare this as an @TeleOp so the app knows to list it as an option in the menu.
public class SimpleOpMode extends OpMode
{
    // make sure your Java class is named SimpleOpMode, or whatever your class is named.
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;
    private boolean lastAButton = false;
    //We let the program know to expect these.
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //This lets us know we have initialized. We will go over Telemetry in the debugging section.

        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        armMotor = hardwareMap.dcMotor.get("arm motor");
        //The names "left motor" and "right motor" must match the motor names on the hardware map.

        leftMotor.setDirection(DcMotor.Direction.FORWARD);  // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        //This makes sure that the motors run in the same direction.
    }
    @Override
    public void loop() {

        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);
        //We set the powers. There isn't much else to do here.

        telemetry.addData("a", "a");
        if( !lastAButton && gamepad1.a){
            // If the A button was not pressed last time...
            // and the A button is pressed this time
            if( armMotor.getPower() > 0){
                //then if the motor was off
                armMotor.setPower(1);
                //turn it on
            } else {
                //and if the motor was on
                armMotor.setPower(0);
                //turn it off
            }
        }
        lastAButton = gamepad1.a;
        //remember the last thing the button was set to for the next time we go through the loop

    }
}
