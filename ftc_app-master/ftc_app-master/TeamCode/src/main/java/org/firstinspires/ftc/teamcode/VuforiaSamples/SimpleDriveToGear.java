package org.firstinspires.ftc.teamcode.VuforiaSamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.TankSamples.SixWheelDrive;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

@Autonomous(name = "Vuforia Drive to Gear", group = "Autonomous")
public class SimpleDriveToGear extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaGetter v = new VuforiaGetter();
        VuforiaTrackables patterns = v.getVuforia(1);
        DcMotor leftFront = hardwareMap.dcMotor.get(UniversalContants.leftFrontDrive);
        DcMotor leftBack = hardwareMap.dcMotor.get(UniversalContants.leftBackDrive);
        DcMotor rightFront = hardwareMap.dcMotor.get(UniversalContants.rightFrontDrive);
        DcMotor rightBack = hardwareMap.dcMotor.get(UniversalContants.rightBackDrive);

        SixWheelDrive drive = new SixWheelDrive(SimpleDriveToGear.this, leftFront, leftBack, rightFront, rightBack);

        waitForStart();

        patterns.activate();

        VuforiaGetter.Pattern pattern = VuforiaGetter.Pattern.Gears;
        double startTime = getRuntime();
        while (opModeIsActive() && !v.getOffset(patterns, pattern).foundValues) {
            drive.setPowers(.6);
        }
        double distance = v.getOffset(patterns, pattern).distance / UniversalContants.millimetersPerInch - 24;
        double horizontal = v.getOffset(patterns, pattern).horizontal / UniversalContants.millimetersPerInch;
        if (distance > 5 || horizontal > 6) {
            double theta = Math.atan2(distance, horizontal);
            double distanceTarget = distance / Math.sin(theta);
            theta *= UniversalContants.degreesPerRadian;
            drive.turnRelative(.25, theta);

            telemetry.addData("Distance Goal", distanceTarget);
            telemetry.update();

            drive.setPowers(.40);
            Thread.sleep((long) distanceTarget * 25);

            drive.turnRelative(.25, -theta);
            while (opModeIsActive() && Math.abs(v.getOffset(patterns, pattern).horizontal) > 3) {
                if (v.getOffset(patterns, pattern).horizontal < 0) {
                    drive.setPowers(.22, -.22);
                } else {
                    drive.setPowers(-.22, .22);
                }
            }
        }
        drive.driveToTargetStraight(VuforiaGetter.Pattern.Gears, patterns, v);
        patterns.deactivate();
    }
}
