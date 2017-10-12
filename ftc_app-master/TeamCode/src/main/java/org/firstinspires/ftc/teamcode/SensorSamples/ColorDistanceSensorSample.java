package org.firstinspires.ftc.teamcode.SensorSamples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class ColorDistanceSensorSample extends OpMode {
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    @Override
    public void init() {
        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(ColorSensor.class, "color distance");
        // We need to enable this LED within the initialization phase.
        // Disabling the LED will allow it to measure

        // get a reference to the distance sensor that shares the same name.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color distance");
    }

    @Override
    public void loop() {
        telemetry.addData("Alpha (Clearness)", colorSensor.alpha());
        telemetry.addData("Red", colorSensor .red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Distance (mm)", distanceSensor.getDistance(DistanceUnit.MM));

        telemetry.update();
    }

}
