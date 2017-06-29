package org.firstinspires.ftc.teamcode.reference;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="ColorSensorLED", group="Autonomous")
@Disabled
public class colorSensorDataWithLED extends LinearOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor color = hardwareMap.get(ColorSensor.class, "c");
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        waitForStart();
        color.enableLed(true);
        while(opModeIsActive()){
            Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);
            telemetry.addData("color.red()", color.red());
            telemetry.addData("color.arbg()", color.argb());
            telemetry.addData("color.green()", color.green());
            telemetry.addData("color.blue()", color.blue());
            telemetry.addData("color.alpha()", color.alpha());
            telemetry.addData("HSV[0]", hsvValues[0]);
            telemetry.addData("HSV[1]", hsvValues[1]);
            telemetry.addData("HSV[2]", hsvValues[2]);

            telemetry.update();
            idle();
        }
    }
}
