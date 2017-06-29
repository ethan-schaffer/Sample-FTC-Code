/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@Autonomous(name="Color Sensors", group="Autonomous")
@Disabled
public class TestColorSensingLine extends LinearOpMode {
    //TWEAKING VALUES
    public static final double BLOCKSERVOOPENVALUE = 0;
    public static final double BLOCKSERVOCLOSEDVALUE = 1;
    public static final double MAXINFEEDPOWER = 1;
    public static final double LEFTSERVOMAXVALUE = 1;
    public static final double LEFTSERVOMINVALUE = .08;
    public static final double RIGHTSERVOMAXVALUE = .94;
    public static final double RIGHTSERVOMINVALUE = 0;

    //GOOD VALUES
    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String BALLBLOCKNAME = "b";//MO Port 3
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 1
    public static final String COLORBOTTOMNAME = "cb";//Port 2

    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlock;
    ColorSensor colorSensorOnBottom, colorSensorOnSide;
    ModernRoboticsI2cRangeSensor range;

    //Runs op mode
    @Override
    public void runOpMode() throws InterruptedException {

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
        colorSensorOnBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorOnBottom.enableLed(true);
        colorSensorOnSide.enableLed(false);
/*
        leftFrontWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(leftFrontWheel.getCurrentPosition()!=0){
        }//wait
        int leftFrontWheelEncoderPosition = leftFrontWheel.getCurrentPosition();
*/
        //WAIT FOR START IS HERE
        waitForStart();

        drive(-.2, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        while( colorSensorOnBottom.alpha() < 4){
            telemetry.addData("A", colorSensorOnBottom.alpha() < 4);
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        drive(.1, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        while( colorSensorOnBottom.alpha() < 4){
            telemetry.addData("A", colorSensorOnBottom.alpha() < 4);
            telemetry.update();
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        //AT FIRST BEACON
        telemetry.addData("Red Value, First pressBeacon", colorSensorOnSide.red());
        telemetry.update();
        if(colorSensorOnSide.red() > colorSensorOnSide.blue()){
            rightButtonPusher.setPosition(LEFTSERVOMAXVALUE);
        } else {
            leftButtonPusher.setPosition(RIGHTSERVOMINVALUE);
        }
        sleep(1000);
/*
        drive(-.2, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        sleep(500);
        leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        while( colorSensorBottom.alpha() < 4) {
        }
        drive(0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        //AT SECOND BEACON
        telemetry.addData("Red Value, Second pressBeacon", colorSensorOnSide.red());
        telemetry.update();
        if(colorSensorOnSide.red() > 3){
            rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
        } else {
            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        }
        sleep(500);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
*/

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