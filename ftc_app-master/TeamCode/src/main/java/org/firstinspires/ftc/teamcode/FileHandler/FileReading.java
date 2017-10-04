package org.firstinspires.ftc.teamcode.FileHandler;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class FileReading extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String FILE_DIR = "/Download";
        boolean lastAButton = gamepad1.a;
        File file = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + FILE_DIR, "route.txt");
        Scanner s;
        try {
            s = new Scanner(file);
            while (s.hasNext()) {
                if (gamepad1.a && !lastAButton) {
                    telemetry.addData("File Said:", s.next());
                }
                lastAButton = gamepad1.a;
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}
