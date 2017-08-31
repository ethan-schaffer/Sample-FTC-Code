package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by ethan on 8/31/17.
 */

class Controllers {
    Gamepad gamepad1, gamepad2;
    Controllers(){}

    void update(LinearOpMode l){
        gamepad1 = l.gamepad1;
        gamepad2 = l.gamepad2;
    }
}
