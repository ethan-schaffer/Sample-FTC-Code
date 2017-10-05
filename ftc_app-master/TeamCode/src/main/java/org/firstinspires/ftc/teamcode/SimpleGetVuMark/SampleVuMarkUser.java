package org.firstinspires.ftc.teamcode.SimpleGetVuMark;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class SampleVuMarkUser extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuMarkGetter vuMarkGetter = new VuMarkGetter();
        VuforiaTrackables trackables = vuMarkGetter.getVuforia();
        waitForStart();

        VuMarkGetter.DistanceOffsets distanceOffsets;
        while (opModeIsActive()) {
            distanceOffsets = vuMarkGetter.getOffset(trackables);
            telemetry.addData("Seeing", distanceOffsets.vuMarkType.toString());
            telemetry.addData("Distance", distanceOffsets.distance);
            telemetry.addData("Horizontal", distanceOffsets.horizontal);
            telemetry.addData("Vertical", distanceOffsets.vertical);
            telemetry.update();
        }

    }
}
