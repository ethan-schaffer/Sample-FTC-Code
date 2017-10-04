package org.firstinspires.ftc.teamcode.AutonomousMenuSample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class AutonomousWithMenu extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize robot
        boolean lastDown = false, lastUp = false;
        int waitTime = 1;
        while (!opModeIsActive()) {
            if (gamepad1.dpad_down && !lastDown) {
                waitTime++;
            } else if (gamepad1.dpad_up && !lastUp) {
                if (waitTime > 0) {
                    waitTime--;
                }
            }
            telemetry.addData("Time Waiting: ", waitTime + " seconds");
            telemetry.update();
            lastDown = gamepad1.dpad_down;
            lastUp = gamepad1.dpad_up;
        }
        // the loop can't end until the "Play" button is pressed

        sleep(waitTime * 1000);

        // do other things

    }
}
