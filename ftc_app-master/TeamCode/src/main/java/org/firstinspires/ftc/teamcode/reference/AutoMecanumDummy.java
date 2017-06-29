/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.reference;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

import java.util.Arrays;

@Disabled
@Autonomous(name="Strafes turnLeft", group="Autonomous")
public class AutoMecanumDummy extends LinearOpMode {

    //Runs op mode
    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our compass
        DcMotor left1, right1, left2, right2;
        ModernRoboticsI2cGyro gyroSensor;
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, "g");
        ColorSensor color = hardwareMap.colorSensor.get("c");
        ModernRoboticsI2cRangeSensor rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
        left1 = hardwareMap.dcMotor.get("l1"); // AL00XR4D.1
        left2 = hardwareMap.dcMotor.get("l2"); // AL00UYRR.1
        right1 = hardwareMap.dcMotor.get("r1");// AL00XR4D.2
        right2 = hardwareMap.dcMotor.get("r2"); // AL00UYRR.2
        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.update();

        // wait for the start button to be pressed
        double startingG = gyroSensor.getHeading();

        left1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(left1.getCurrentPosition()!=0){
        }//wait
        int encPos = left1.getCurrentPosition();

        //WAIT FOR START IS HERE
        waitForStart();

        resetEncs(left1, right1, left2, right2);
        telemetry.addData("turnLeft", encPos);
        telemetry.update();
        drive(-.65, left1, right1, left2, right2);
        while(Math.abs((left1.getCurrentPosition()-encPos)) < 3000) {
            telemetry.addData("turnLeft", left1.getCurrentPosition() - encPos);
            telemetry.update();
        }
        drive(0, left1, right1, left2, right2);
        while(rangeSensor.getDistance(DistanceUnit.CM) > 12){
            //strafes LEFT
            double  val = .7;
            left1.setPower(val);
            left2.setPower(-val);
            right1.setPower(-val);
            right2.setPower(val);
            telemetry.addData("raw ultrasonic", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        drive(0, left1, right1, left2, right2);
        telemetry.update();
        encPos = left1.getCurrentPosition();
        drive(.3, left1, right1, left2, right2);
        while(Math.abs((left1.getCurrentPosition()-encPos)) < 300) {
            telemetry.addData("turnLeft", left1.getCurrentPosition() - encPos);
            telemetry.update();
        }

        while(Math.abs(gyroSensor.getHeading()-startingG) > 5){
            double val = .2;
            left1.setPower(val);
            left2.setPower(-val);
            right1.setPower(-val);
            right2.setPower(val);
        }
        drive(0, left1, right1, left2, right2);

        while( ((color.red()+color.blue()+color.green())/3) < 2){
            drive(-.2, left1, right1, left2, right2);
        }
        drive(0, left1, right1, left2, right2);
        sleep(1000);
        drive(-.2, left1, right1, left2, right2);
        sleep(500);
        while( ((color.red()+color.blue()+color.green())/3) < 2) {
        }
        drive(0, left1, right1, left2, right2);
    }

    //Constant Power
    public static void drive(double val, DcMotor left1, DcMotor right1, DcMotor left2, DcMotor right2){
        left1.setPower(val);
        left2.setPower(val);
        right1.setPower(val);
        right2.setPower(val);
    }

    //Mecanum:
    // y - forwards
    // x - side
    // c - rotation
    public static void arcade(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }

    //resets encoders
    public static void resetEncs(DcMotor left1, DcMotor right1, DcMotor left2, DcMotor right2){
        while(left1.getCurrentPosition() != 0){
            left1.setMode(DcMotor.RunMode.RESET_ENCODERS);
            left2.setMode(DcMotor.RunMode.RESET_ENCODERS);
            right1.setMode(DcMotor.RunMode.RESET_ENCODERS);
            right2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}