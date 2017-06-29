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

/**
 * Created by Ethan Schaffer on 11/8/2016.
 */

@Autonomous(name="Auto (2 Color)", group="Autonomous")
@Disabled
public class doubleFloorColor extends LinearOpMode {
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
    public static final String COLORLEFTBOTTOMNAME = "cb";//Port 2
    public static final String COLORRIGHTBOTTOMNAME = "cb2"; //Port 4

    public static final double LEFT_SERVO_OFF_VALUE = 0;
    public static final double LEFT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_OFF_VALUE = 0;
    public static final double BALLBLOCKOPEN = 0;
    public static final double BALLBLOCKCLOSED = 1;


    //TODO: make a pretty 'map'

    // Shoot
    public static final double POWER1 = -.5, DISTANCE1 = 1900;           //Distance Forwards before shot
    public static final long SHOTTIMEINMILLISECONDS = 1000;              //How long we shoot the ball
//    public static final long SHOT_TIME_IN_MILLISECONDS = 4500;              //How long we shoot the ball

    //Line up with wall
    public static final double POWER2 = .2,  DISTANCE2 = 500;             //Backup Distance
    public static final double POWER3 = .2,  GYRO3 = 32; //Turn Reading
    public static final double POWER4 = -.5, DISTANCE4 = 3200;            //Forwards Distance
    public static final double POWER5 = -.2, GYRO5READINGTARGET = 5; //Turn Reading

    //Allign with wall by running into it
    public static final double POWER6 = .45, DISTANCE6 = 200;    //Strafe to between the white lines.
    public static final double CM_FROM_WALL_VALUE = 4;

    //Getting Beacons
    public static final double POWER7 = .25, COLOR_READING_FOR_LINE = 4; //Backwards to beacon #1
    public static final double POWER8 = -.25; //Forwards to pressBeacon #2

    //Cap Ball
    public static  final double POWER9 = .2, GYRO9 = 45; //ReOrient for Hitting Cap Ball
    public static final double POWER10 = .5, DISTANCE10 = 2750; //Distance to Ball
    public static final double POWER_HANDLE_COLOR = .2;

    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlock;
    ColorSensor colorSensorLeftBottom, colorSensorRightBottom, colorSensorOnSide;
    ModernRoboticsI2cRangeSensor range;
    ModernRoboticsI2cGyro gyroSensor;

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
        leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        colorSensorLeftBottom = hardwareMap.colorSensor.get(COLORLEFTBOTTOMNAME);
        colorSensorRightBottom = hardwareMap.colorSensor.get(COLORRIGHTBOTTOMNAME);
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorLeftBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorRightBottom.setI2cAddress(I2cAddr.create8bit(0x2c));
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRONAME);
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){
            telemetry.addData("Gyro", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Calibrated");
        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlock.setPosition(BALLBLOCKCLOSED);

        waitForStart();

        resetEncoder(leftFrontWheel);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE1) {
            arcade(POWER1, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        ballBlock.setPosition(BALLBLOCKOPEN);
        sleep(50);
        shoot1.setPower(1);
        shoot2.setPower(1);
        sleep(100);
        infeed.setPower(1);
        sleep(SHOTTIMEINMILLISECONDS);
        infeed.setPower(0);
        shoot1.setPower(0);
        shoot2.setPower(0);
        ballBlock.setPosition(BALLBLOCKCLOSED);

/*
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE1PART2) {
            arcade(POWER_TO_SHOT, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopDriveMotors();
*/

        //Back Up
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE2) {
            arcade(POWER2, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        //TURN
        while(gyroSensor.getIntegratedZValue() < GYRO3){
            arcade(0, 0, POWER3, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        //Forwards
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE4) {
            arcade(POWER4, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        //TURN BACK
        while( gyroSensor.getIntegratedZValue() > GYRO5READINGTARGET){
            telemetry.addData("Gyro", gyroSensor.getIntegratedZValue());
            telemetry.addData("G Targ", GYRO5READINGTARGET);
            telemetry.update();
            arcade(0, 0, POWER5, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        //strafe to wall

        sleep(50);
        while(range.getDistance(DistanceUnit.CM) > CM_FROM_WALL_VALUE) {
            telemetry.addData("Dist", range.getDistance(DistanceUnit.CM));
            telemetry.addData("D Targ", CM_FROM_WALL_VALUE);
            telemetry.update();
            arcade(0, POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE6) {
            arcade(0, POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < 2* DISTANCE6) {
            arcade(0, -POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

/*
  /\ /\  __          ______  _____  _  __ _____   /\ /\
 |/\|/\| \ \        / / __ \|  __ \| |/ // ____| |/\|/\|
          \ \  /\  / / |  | | |__) | ' /| (___
           \ \/  \/ /| |  | |  _  /|  <  \___ \
            \  /\  / | |__| | | \ \| . \ ____) |
             \/  \/   \____/|_|  \_\_|\_\_____/
*/
        //backwards to back color voltage_sensor
        sleep(50);
        while(colorSensorLeftBottom.alpha() < 4) {
            arcade(POWER7, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        sleep(100);
        //press buttons
/*
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE_TO_HIT_WALL) {
            arcade(0, POWER_HIT_WALL, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopDriveMotors();
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < 2* DISTANCE_TO_HIT_WALL) {
            arcade(0, -POWER_HIT_WALL, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopDriveMotors();
*/
        handleColor(-POWER_HANDLE_COLOR);
        sleep(100);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);

        arcade(POWER8, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        sleep(500);
        ///forwards to front color voltage_sensor
        while(colorSensorLeftBottom.alpha() < COLOR_READING_FOR_LINE) {
            arcade(POWER8, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        sleep(100);
        //press buttons
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE6) {
            arcade(0, POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < 2* DISTANCE6) {
            arcade(0, -POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        handleColor(POWER_HANDLE_COLOR);
        sleep(100);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);

        //TURN BACK
        while(gyroSensor.getIntegratedZValue() < GYRO9){
            arcade(0, 0, POWER9, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();

        //Back Up
        resetEncoder(leftFrontWheel);
        sleep(50);
        while(Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE10) {
            arcade(POWER10, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
            idle();
        }
        stopMotors();


    }

    public void handleColor(double power) {
        while(colorSensorLeftBottom.alpha() < 4) {
            arcade(power, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }
        stopMotors();

        if(colorSensorOnSide.blue() > colorSensorOnSide.red()){
            rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        } else {
            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
        }
    }

    public static void resetEncoder(DcMotor m){
        m.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(m.getCurrentPosition()!=0){
        }//wait
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopMotors(){
        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);
    }

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
}
