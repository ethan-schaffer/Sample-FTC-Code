package org.firstinspires.ftc.teamcode.reference;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

import static android.R.attr.data;
import static android.R.attr.hand_hour;

/**
 * Created by Ethan Schaffer on 1/31/2017.
 */
@TeleOp(group = "TeleOp", name = "RPM Tester")
@Disabled
public class rpmTester extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor s1, s2, in;
        String[] logEntries;
        String FILE_DIR = "/Download";
        File file = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + FILE_DIR, "rpm-testing.txt");
        s1 = hardwareMap.dcMotor.get("sh1");
        s2 = hardwareMap.dcMotor.get("sh2");
        in = hardwareMap.dcMotor.get("in");
        in.setDirection(DcMotorSimple.Direction.REVERSE);
        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        int pastEnc1 = s1.getCurrentPosition(), pastEnc2 = s2.getCurrentPosition();
        int deltaEnc1 = 0, deltaEnc2 = 0;
        double rpm1 = 0, rpm2 = 0, percentError = 0;
        String data1 = "", data2 = "", data3 = "", data4 = "";
        waitForStart();
        s1.setPower(.55);
        s2.setPower(.55);
        double DeltaAvg = 0;
        double targPerSecond = 1800;
        double timeWait = .33; //in seconds
        /*
        * PWR  Made/Shot
        * .2   4   / 10
        * .33  9   / 10
        * .5   9   / 10
        *  1   10  / 10
        * */
        double target = targPerSecond * timeWait;
        while(opModeIsActive()) {
            in.setPower(1);
            if(gamepad1.x){
                target+=100;
            } else if(gamepad1.b){
                target-=100;
            }
            deltaEnc1 = Math.abs(Math.abs(s1.getCurrentPosition())-pastEnc1);
            deltaEnc2 = Math.abs(Math.abs(s2.getCurrentPosition())-pastEnc2);
            DeltaAvg = (deltaEnc1 + deltaEnc2) / 2;
            percentError = ((DeltaAvg - target) / DeltaAvg);

//            if (DeltaAvg < target + threshold) {
                s1.setPower(Range.clip( s1.getPower() - percentError / 5 , -1, 1));
                s2.setPower(Range.clip( s1.getPower() - percentError / 5 , -1, 1));
//            } else if (DeltaAvg > target - threshold) {
//                s1.setPower(s1.getPower() + Range.clip( (DeltaAvg - target) / target, .01, .05) );
//                s2.setPower(s1.getPower() + Range.clip( (DeltaAvg - target) / target, .01, .05) );
//            }
            telemetry.clear();
            telemetry.addData("Target", target);
            telemetry.addData("Power", s1.getPower());
            telemetry.addData("Delta 1", deltaEnc1);
            telemetry.addData("Delta 2", deltaEnc2);
            telemetry.addData("Delta Avg", DeltaAvg);
//            telemetry.addData("Position 1", s1.getCurrentPosition());
//            telemetry.addData("Position 2", s2.getCurrentPosition());
            telemetry.update();
            pastEnc1 = Math.abs(s1.getCurrentPosition());
            pastEnc2 = Math.abs(s2.getCurrentPosition());
            sleep((long)(timeWait * 1000));
        }
    }


}
