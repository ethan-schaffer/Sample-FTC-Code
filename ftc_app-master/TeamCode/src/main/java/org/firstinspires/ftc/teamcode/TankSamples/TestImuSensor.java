package org.firstinspires.ftc.teamcode.TankSamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

@Autonomous(name = "TestImuSensor")
public class TestImuSensor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get(UniversalContants.leftFrontDrive);
        DcMotor leftBack = hardwareMap.dcMotor.get(UniversalContants.leftBackDrive);
        DcMotor rightFront = hardwareMap.dcMotor.get(UniversalContants.rightFrontDrive);
        DcMotor rightBack = hardwareMap.dcMotor.get(UniversalContants.rightBackDrive);

        SixWheelDrive drive = new SixWheelDrive(TestImuSensor.this, leftFront, leftBack, rightFront, rightBack);
        waitForStart();
        drive.turnExact(.25, 90);
        Thread.sleep(1000);
        drive.turnExact(.25, -90);
        Thread.sleep(1000);
        drive.turnRelative(.25, 90);
    }

}
