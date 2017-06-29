package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.reference.Robot;

/**
 * Created by Ethan Schaffer on 1/6/2017.
 */
@Autonomous(name="Test Object", group="Autonomous")
@Disabled
public class testingRobotObj extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot r = new Robot(hardwareMap);
        r.initialize(telemetry);
        waitForStart();
        r.Move(35, 1.00);
    }
}
