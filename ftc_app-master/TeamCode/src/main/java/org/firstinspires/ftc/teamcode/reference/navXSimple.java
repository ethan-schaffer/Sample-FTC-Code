package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

/**
 * Created by Ethan Schaffer on 9/24/2016.
 */
@Autonomous(name="navX", group="zSensor Testing")
@Disabled
public class navXSimple extends LinearOpMode {
    AHRS navX;
    DeviceInterfaceModule dim;

    @Override
    public void runOpMode() throws InterruptedException {
        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");
        navX = new AHRS(dim, 5, AHRS.DeviceDataType.kProcessedData, (byte)50);
        navX.zeroYaw();
        int timer = 0;
        int cycler = 0;
        while (navX.isCalibrating() && timer < 1000) {
            timer++;
            if (timer % 15 == 0) {
                cycler++;
            }
            if (cycler == 0) {
                telemetry.addData("Gyro", " is still Calibrating");
            } else if (cycler == 1) {
                telemetry.addData("Gyro", " is still Calibrating.");
            } else if (cycler == 2) {
                telemetry.addData("Gyro", " is still Calibrating..");
            } else {
                cycler = 0;
                telemetry.addData("Gyro", " is still Calibrating...");
            }
            telemetry.update();
        } //This silly looking code above animates a "..." sequence in telemetry, if the gyroscope is still calibrating
        telemetry.addData("Gyro", "Done!");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Yaw", navX.getYaw());
            telemetry.update();
            idle();
        }
    }
}
