package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Andrew on 12/8/2016.
 */

@Autonomous(name="RPMTest2", group="Testing")
@Disabled
public class RPMTest2 extends LinearOpMode
{
    DcMotor shooter1, shooter2, infeed;

    public void runOpMode() throws InterruptedException
    {
        shooter1 = hardwareMap.dcMotor.get("sh1");
        shooter2 = hardwareMap.dcMotor.get("sh2");
        infeed = hardwareMap.dcMotor.get("in");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setPower(1);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setPower(1);
        infeed.setPower(-1);
        long timeelapsed = 100;
        while(opModeIsActive())
        {
            int original1 = shooter1.getCurrentPosition();
            int original2 = shooter2.getCurrentPosition();
            Thread.sleep(timeelapsed);
            int after1 = shooter1.getCurrentPosition();
            int after2 = shooter2.getCurrentPosition();

            int ticks1 = Math.abs(after1 - original1);
            int ticks2 = Math.abs(after2 - original2);

            int Rotations1 = ticks1/(28*4);
            int Rotations2 = ticks2/(28*4);

            int rpm1 = Rotations1*(1000/(int)timeelapsed)*60;
            int rpm2 = Rotations2*(1000/(int)timeelapsed)*60;

            telemetry.addData("rpm", rpm1);
            telemetry.addData("rpm2", rpm2);
            telemetry.update();

        }
        shooter1.setPower(0);
    }

}
