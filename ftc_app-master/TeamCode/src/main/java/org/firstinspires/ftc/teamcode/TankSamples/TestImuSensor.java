package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ethan Schaffer on 7/3/2017.
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
