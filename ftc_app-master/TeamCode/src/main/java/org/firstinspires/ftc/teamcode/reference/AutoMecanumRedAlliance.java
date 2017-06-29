/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Autonomous(name="Red Alliance", group="Autonomous")
@Disabled
public class AutoMecanumRedAlliance extends LinearOpMode {
    //TWEAKING VALUES
    public static final double BLOCKSERVOOPENVALUE = 0;
    public static final double BLOCKSERVOCLOSEDVALUE = 1;
    public static final double DISTANCEBEFORESHOT = 1500;
    public static final double DISTANCEBACKPOSTSHOT = -300;
    public static final double DISTANCESTRAFEPOSTSHOT = 500;
    public static final double DISTANCEPOSTSTRAFEPOSTSHOT = 750;

    public static final double GYROOFFSETDIVIDEFACTOR = 360;

    public enum AllianceColors {
        RED, BLUE
    }
    public AllianceColors alliance = AllianceColors.RED;

    //GOOD VALUES
    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String GYRONAME = "g"; //Port 4
    public static final String BALLBLOCKNAME = "b";//MO Port 3
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 1
    public static final String COLORBOTTOMNAME = "cb";//Port 2
    public static final double MAXINFEEDPOWER = 1;
    public static final double LEFTSERVOMAXVALUE = .75;
    public static final double LEFTSERVOMINVALUE = .08;
    public static final double RIGHTSERVOMAXVALUE = .94;
    public static final double RIGHTSERVOMINVALUE = .25;
    public static double STRAFEFACTOR = 1;
    public static double FORWARDSFACTOR = 1;

    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlock;
    ColorSensor colorSensorOnBottom, colorSensorOnSide;
    ModernRoboticsI2cRangeSensor range;
    ModernRoboticsI2cGyro gyroSensor;

    //Runs op mode
    @Override
    public void runOpMode() throws InterruptedException {
        if(alliance == AllianceColors.BLUE){
            FORWARDSFACTOR = -FORWARDSFACTOR; // Auto-Toggle the FORWARDSFACTOR is we are on the blue alliance
        }
        leftFrontWheel= hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel=hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel= hardwareMap.dcMotor.get(RIGHT2NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1=hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2=hardwareMap.dcMotor.get(SHOOT2NAME);
        infeed=hardwareMap.dcMotor.get(INFEEDNAME);
        infeed.setDirection(DcMotorSimple.Direction.REVERSE);
        ballBlock=hardwareMap.servo.get(BALLBLOCKNAME);
        leftButtonPusher =hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher =hardwareMap.servo.get(RIGHTPUSHNAME);
        leftButtonPusher.setPosition(LEFTSERVOMAXVALUE);
        rightButtonPusher.setPosition(RIGHTSERVOMINVALUE);
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        colorSensorOnBottom = hardwareMap.colorSensor.get(COLORBOTTOMNAME);
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorOnBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRONAME);
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){
            telemetry.addData("Gyro", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Calibrated");
        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();


        /*                _ _    __               _             _
                       (_) |  / _|             | |           | |
         __      ____ _ _| |_| |_ ___  _ __ ___| |_ __ _ _ __| |_
         \ \ /\ / / _` | | __|  _/ _ \| '__/ __| __/ _` | '__| __|
          \ V  V / (_| | | |_| || (_) | |  \__ \ || (_| | |  | |_
           \_/\_/ \__,_|_|\__|_| \___/|_|  |___/\__\__,_|_|   \__|
        */
        waitForStart();
        leftFrontWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(leftFrontWheel.getCurrentPosition()!=0){
        }//wait
        int leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();
        double gyroReadingTarget = gyroSensor.getIntegratedZValue();
        if(alliance == AllianceColors.BLUE){
            gyroReadingTarget+=180;
            gyroReadingTarget = gyroReadingTarget%360;
        }
        /*
                _ _                _               _                 _
               | (_)              | |             | |               | |
           __ _| |_  __ _ _ __    | |_ ___     ___| |__   ___   ___ | |_
          / _` | | |/ _` | '_ \   | __/ _ \   / __| '_ \ / _ \ / _ \| __|
         | (_| | | | (_| | | | |  | || (_) |  \__ \ | | | (_) | (_) | |_
          \__,_|_|_|\__, |_| |_|   \__\___/   |___/_| |_|\___/ \___/ \__|
                     __/ |
                    |___/
        */
        resetEncs(leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        telemetry.addData("turnLeft", leftFrontWheelEncoderPosition);
        telemetry.update();
        drive(-.85, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        while(Math.abs((leftFrontWheel.getCurrentPosition()-leftFrontWheelEncoderPosition)) < DISTANCEBEFORESHOT) {
            telemetry.addData("turnLeft Encoder", leftFrontWheel.getCurrentPosition() - leftFrontWheelEncoderPosition);
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        /*
              _                 _
             | |               | |
          ___| |__   ___   ___ | |_
         / __| '_ \ / _ \ / _ \| __|
         \__ \ | | | (_) | (_) | |_
         |___/_| |_|\___/ \___/ \__|
         */
        ballBlock.setPosition(BLOCKSERVOOPENVALUE); //Make sure to update
        shoot1.setPower(1); shoot2.setPower(1);
        infeed.setPower(MAXINFEEDPOWER);
        sleep(1500);//can adjust
        shoot1.setPower(0); shoot2.setPower(0);
        ballBlock.setPosition(BLOCKSERVOCLOSEDVALUE); //Make sure to update
        infeed.setPower(0);

        /*
           __                                 _
          / _|                               | |
         | |_ ___  _ ____      ____ _ _ __ __| |___   _ __ ___   ___  _ __ ___
         |  _/ _ \| '__\ \ /\ / / _` | '__/ _` / __| | '_ ` _ \ / _ \| '__/ _ \
         | || (_) | |   \ V  V / (_| | | | (_| \__ \ | | | | | | (_) | | |  __/
         |_| \___/|_|    \_/\_/ \__,_|_|  \__,_|___/ |_| |_| |_|\___/|_|  \___|
        */
        drive(.45, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        while(Math.abs((leftFrontWheel.getCurrentPosition()-leftFrontWheelEncoderPosition)) > DISTANCEBEFORESHOT + DISTANCEBACKPOSTSHOT) {
            telemetry.addData("turnLeft Encoder", leftFrontWheel.getCurrentPosition() - leftFrontWheelEncoderPosition);
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        /*
              _              __       _                        _ _
             | |            / _|     | |                      | | |
          ___| |_ _ __ __ _| |_ ___  | |_ ___   __      ____ _| | |
         / __| __| '__/ _` |  _/ _ \ | __/ _ \  \ \ /\ / / _` | | |
         \__ \ |_| | | (_| | ||  __/ | || (_) |  \ V  V / (_| | | |
         |___/\__|_|  \__,_|_| \___|  \__\___/    \_/\_/ \__,_|_|_|
         */
        leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();
        while(Math.abs(leftFrontWheel.getCurrentPosition() + leftFrontWheelEncoderPosition) < DISTANCESTRAFEPOSTSHOT){
            double val = .5;
            leftFrontWheel.setPower(val);
            leftBackWheel.setPower(-val);
            rightFrontWheel.setPower(-val);
            rightBackWheel.setPower(val);
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        sleep(100);

        resetEncs(leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel);
        leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();
        while(Math.abs(leftFrontWheel.getCurrentPosition() + leftFrontWheelEncoderPosition) < DISTANCEPOSTSTRAFEPOSTSHOT){
            double  val = -.5;
            leftFrontWheel.setPower(val);
            leftBackWheel.setPower(val);
            rightFrontWheel.setPower(val);
            rightBackWheel.setPower(val);
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        sleep(100);

        while(range.getDistance(DistanceUnit.CM) > 12){
            //strafes LEFT
            double  val = .5;
            leftFrontWheel.setPower(val*STRAFEFACTOR);
            leftBackWheel.setPower(-val*STRAFEFACTOR);
            rightFrontWheel.setPower(-val*STRAFEFACTOR);
            rightBackWheel.setPower(val*STRAFEFACTOR);
            telemetry.addData("raw ultrasonic", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        telemetry.update();

        /*
           __ _              _ _                                  _
          / _(_)            | (_)                                | |
         | |_ ___  __   __ _| |_  __ _ _ __  _ __ ___   ___ _ __ | |_
         |  _| \ \/ /  / _` | | |/ _` | '_ \| '_ ` _ \ / _ \ '_ \| __|
         | | | |>  <  | (_| | | | (_| | | | | | | | | |  __/ | | | |_
         |_| |_/_/\_\  \__,_|_|_|\__, |_| |_|_| |_| |_|\___|_| |_|\__|
                                  __/ |
                                 |___/
         */
/*        leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();
        drive(.3*FORWARDSFACTOR, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        while(Math.abs((leftFrontWheel.getCurrentPosition()-leftFrontWheelEncoderPosition)) < 500) {
            telemetry.addData("turnLeft", leftFrontWheel.getCurrentPosition() - leftFrontWheelEncoderPosition);
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
*/
        leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();
        while(Math.abs((leftFrontWheel.getCurrentPosition()-leftFrontWheelEncoderPosition)) < 150){
            //strafes LEFT
            double  val = -.2;
            leftFrontWheel.setPower(val*STRAFEFACTOR);
            leftBackWheel.setPower(-val*STRAFEFACTOR);
            rightFrontWheel.setPower(-val*STRAFEFACTOR);
            rightBackWheel.setPower(val*STRAFEFACTOR);
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        /*
          _ _
         | (_)
         | |_ _ __   ___   _   _ _ __
         | | | '_ \ / _ \ | | | | '_ \
         | | | | | |  __/ | |_| | |_) |
         |_|_|_| |_|\___|  \__,_| .__/
                                | |
                                |_|
         */
/*
        while(gyroSensor.getIntegratedZValue() != gyroReadingTarget){
            double power = Math.abs(gyroSensor.getIntegratedZValue() - gyroReadingTarget)/GYROOFFSETDIVIDEFACTOR;
            if(gyroSensor.getIntegratedZValue() > gyroReadingTarget){
                leftFrontWheel.setPower(power);
                leftBackWheel.setPower(power);
                rightFrontWheel.setPower(-power);
                rightBackWheel.setPower(-power);
            }
            else {
                leftFrontWheel.setPower(-power);
                leftBackWheel.setPower(-power);
                rightFrontWheel.setPower(power);
                rightBackWheel.setPower(power);
            }
        }
*/
        drive(-.2*FORWARDSFACTOR, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        while( colorSensorOnBottom.alpha() < 3){
            telemetry.addData("Alpha", colorSensorOnBottom.alpha());
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        //Go back to combat overshoot
        drive(.1*FORWARDSFACTOR, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        while( colorSensorOnBottom.alpha() < 3) {
            telemetry.addData("Alpha", colorSensorOnBottom.alpha());
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);


        //AT FIRST BEACON
        /*
          _ __  _ __ ___  ___ ___
         | '_ \| '__/ _ \/ __/ __|
         | |_) | | |  __/\__ \__ \
         | .__/|_|  \___||___/___/
         | |
         |_|
         */
        telemetry.addData("Red Value, First pressBeacon", colorSensorOnSide.red());
        if(colorSensorOnSide.red() < colorSensorOnSide.blue()){
            rightButtonPusher.setPosition(RIGHTSERVOMAXVALUE);
            leftButtonPusher.setPosition(LEFTSERVOMAXVALUE);
        } else {
            leftButtonPusher.setPosition(LEFTSERVOMINVALUE);
            rightButtonPusher.setPosition(RIGHTSERVOMINVALUE);
        }
        sleep(1000);

        /*
          _ _
         | (_)
         | |_ _ __   ___   _   _ _ __
         | | | '_ \ / _ \ | | | | '_ \
         | | | | | |  __/ | |_| | |_) |
         |_|_|_| |_|\___|  \__,_| .__/
                                | |
                                |_|
         */
        drive(-.2*FORWARDSFACTOR, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        sleep(500);
        leftButtonPusher.setPosition(LEFTSERVOMAXVALUE);
        rightButtonPusher.setPosition(RIGHTSERVOMINVALUE);
        while( colorSensorOnBottom.alpha() < 3) {
            telemetry.addData("Alpha", colorSensorOnBottom.alpha());
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        //Go back to combat overshoot
        drive(.1*FORWARDSFACTOR, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        while( colorSensorOnBottom.alpha() < 3) {
            telemetry.addData("Alpha", colorSensorOnBottom.alpha());
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        //AT SECOND BEACON
        /*
          _ __  _ __ ___  ___ ___
         | '_ \| '__/ _ \/ __/ __|
         | |_) | | |  __/\__ \__ \
         | .__/|_|  \___||___/___/
         | |
         |_|
         */
        telemetry.addData("Red Value, Second pressBeacon", colorSensorOnSide.red());
        if(colorSensorOnSide.red() < colorSensorOnSide.blue()){
            rightButtonPusher.setPosition(RIGHTSERVOMAXVALUE);
            leftButtonPusher.setPosition(LEFTSERVOMAXVALUE); //make sure it doesn't press the wrong button
        } else {
            leftButtonPusher.setPosition(LEFTSERVOMINVALUE);
            rightButtonPusher.setPosition(RIGHTSERVOMINVALUE); //make sure it doesn't press the wrong button
        }
        sleep(500);
        leftButtonPusher.setPosition(LEFTSERVOMAXVALUE); //disable servos
        rightButtonPusher.setPosition(RIGHTSERVOMINVALUE); //disable servos

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