package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="Sensors", group="Autonomous")
@Disabled
public class SensorTesting extends LinearOpMode{

    ColorSensor color_bottom, color_bottom2, color_side;
    ModernRoboticsI2cRangeSensor rangeSensor;
    DeviceInterfaceModule dim;
    @Override
    public void runOpMode() throws InterruptedException {
        color_bottom = hardwareMap.colorSensor.get("cb");
        color_bottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        color_side = hardwareMap.colorSensor.get("cs");
        color_side.setI2cAddress(I2cAddr.create8bit(0x3c));
        color_bottom.enableLed(true);
        color_side.enableLed(false);
        dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");


        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("bottom red", color_bottom.red());
            telemetry.addData("bottom green", color_bottom.green());
            telemetry.addData("bottom blue", color_bottom.blue());
            telemetry.addData("bottom alpha", color_bottom.alpha());
            telemetry.addData("-----", "-----------");
            telemetry.addData("side red", color_side.red());
            telemetry.addData("side green", color_side.green());
            telemetry.addData("side blue", color_side.blue());
            if(color_side.red() > color_side.blue()){
                dim.setLED(0, false);
                dim.setLED(1, true);
            } else if(color_side.blue() > color_side.red()){
                dim.setLED(1, false);
                dim.setLED(0, true);
            } else {
                dim.setLED(0, false);
                dim.setLED(1, false);
            }

            telemetry.addData("-- --", "-----------");
            telemetry.addData("Range CM", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Range Optical", rangeSensor.cmOptical());

            telemetry.update();
            idle();
        }
    }
}
