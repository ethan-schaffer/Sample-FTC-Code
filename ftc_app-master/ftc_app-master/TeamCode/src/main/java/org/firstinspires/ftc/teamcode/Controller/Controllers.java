package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by ethan on 8/31/17.
 */
class ControllerMock {
    public boolean a, b, x, y, left_stick_button, right_stick_button, dpad_left, dpad_right, dpad_up, dpad_down, left_bumper, right_bumper;
    public float left_trigger, right_trigger, left_stick_x, left_stick_y, right_stick_x, right_stick_y;
    void update(Gamepad gamepad){
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        left_stick_button = gamepad.left_stick_button;
        right_stick_button = gamepad.right_stick_button;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        dpad_down = gamepad.dpad_down;
        dpad_up = gamepad.dpad_up;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
    }
}
class Controllers {
    ControllerMock gamepad1, gamepad2;
    Controllers(){}

    void update(LinearOpMode l){
        gamepad1.update(l.gamepad1);
        gamepad2.update(l.gamepad2);
    }
}
