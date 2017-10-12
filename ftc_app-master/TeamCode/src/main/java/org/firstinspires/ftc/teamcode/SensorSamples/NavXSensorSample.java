package org.firstinspires.ftc.teamcode.SensorSamples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class NavXSensorSample extends LinearOpMode {
    // This program uses a unique LinearOpMode implementation, rather than the OpMode implementation which can be found in other samples.
    // This is because a LinearOpMode gives us access to the isStopRequested() method, which we want while calibrating the NavX.
    // Please see the comment on lines 36-41 for more information about why this choice was made.
    AHRS navx;

    @Override
    public void runOpMode() throws InterruptedException {
        int navXPort = hardwareMap.i2cDevice.get("navx").getPort();
        // get a reference to the port using a reference like this.
        // this allows us to change the port

        byte frequencyHz = (byte) 100;
        navx = new AHRS(hardwareMap.deviceInterfaceModule.get("dim"), navXPort, AHRS.DeviceDataType.kProcessedData, frequencyHz);
        navx.zeroYaw();
        while (navx.isCalibrating() && !isStopRequested()) {
            // The navX can sometimes fail to calibrate properly.
            // If the calibration takes longer than a few seconds, a user should press the "stop" button on their driver station.
            // If this happens, teams will likely see an "OpMode stuck in loop(). Restarting robot controller app." error.
            // So, we can add the "&& !isStopRequested()" to the while loop.
            // Thus, if a stop is requested, the while loop will finish and the opMode will end.
            // Teams can then restart their robot and recalibrate their NavX.

            telemetry.addLine("Calibrating...");
            telemetry.addData("Yaw", navx.getYaw());
            if (!navx.isConnected()) {
                telemetry.addLine("NavX is Disconnected!");
                telemetry.update();
            }
        }
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Yaw (Heading)", navx.getYaw());
        }

        telemetry.update();

    }
}