package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="Sensors", group="zSensor Testing")
public class SensorInfo extends LinearOpMode{
    Robot r = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {

        r.initialize(SensorInfo.this, hardwareMap, telemetry, false);
        while(!isStopRequested() && !isStarted()){
//            telemetry.addData("Yaw", r.navX.getYaw());
//            telemetry.addData("Pitch", r.navX.getPitch());
//            telemetry.addData("Roll", r.navX.getRoll());
            telemetry.addData("___", "___");
            telemetry.addData("Red", r.colorSensorOnSide.red());
            telemetry.addData("Blue", r.colorSensorOnSide.blue());
            telemetry.addData("Range", r.range.getDistance(DistanceUnit.CM));
            telemetry.addData("---", "---");
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){
        /*
            telemetry.addData("Yaw", r.navX.getYaw());
            telemetry.addData("Pitch", r.navX.getPitch());
            telemetry.addData("Roll", r.navX.getRoll());
            telemetry.addData("___", "___");
        */
            telemetry.addData("Red", r.colorSensorOnSide.red());
            telemetry.addData("Blue", r.colorSensorOnSide.blue());
            telemetry.addData("Range", r.range.getDistance(DistanceUnit.CM));
            telemetry.addData("---", "---");
            telemetry.update();
        }
    }
}
