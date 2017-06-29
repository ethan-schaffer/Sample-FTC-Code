package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Andrew on 12/8/2016.
 */

@Autonomous(name="RPMTest", group="Testing")
@Disabled
public class RPMTest extends LinearOpMode
{
    DcMotor Motor;

    public void runOpMode() throws InterruptedException
    {
        Motor = hardwareMap.dcMotor.get("sh1");
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor.setPower(1);

        while(opModeIsActive())
        {
            int Original = Motor.getCurrentPosition();
            Thread.sleep(1000);
            int After = Motor.getCurrentPosition();
            int Ticks = After - Original;
            int Rotations = Ticks/(28*4);
            int RPM = Rotations*60;

            telemetry.addData("RPM", RPM);
            telemetry.update();

        }
        Motor.setPower(0);
    }

}
