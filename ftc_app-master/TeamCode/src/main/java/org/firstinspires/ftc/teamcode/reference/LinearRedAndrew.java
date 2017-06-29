package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Andrew on 11/22/2016.
 * Copyright © 2016 Andrew Gunawan
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Andrew's Linear Red", group="Autonomous")
@Disabled
public class LinearRedAndrew extends LinearOpMode
{
    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlockRight, ballBlockLeft;
    ColorSensor colorSensorOnSide, colorSensorLeftBottom, colorSensorRightBottom;
    ModernRoboticsI2cGyro gyroSensor;
    DeviceInterfaceModule dim;
    ModernRoboticsI2cRangeSensor range;

    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String BALLBLOCKLEFTNAME = "bl", BALLBLOCKRIGHTNAME = "br"; //MO Ports 3+4
    public static final double BALLBLOCKLEFTOPEN = 1, BALLBLOCKLEFTCLOSED = 0;
    public static final double BALLBLOCKRIGHTOPEN = 0, BALLBLOCKRIGHTCLOSED = 1;
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 1
    public static final String COLORLEFTBOTTOMNAME = "cb";//Port 2
    public static final String COLORRIGHTBOTTOMNAME = "cb2"; //Port 4
    public static final String GYRONAME = "g"; //Port 4

    public void runOpMode() throws InterruptedException
    {
        leftFrontWheel = hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(RIGHT2NAME);
        rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        shoot1 = hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2 = hardwareMap.dcMotor.get(SHOOT2NAME);
        infeed = hardwareMap.dcMotor.get(INFEEDNAME);
        infeed.setDirection(DcMotorSimple.Direction.REVERSE);

        ballBlockRight = hardwareMap.servo.get(BALLBLOCKRIGHTNAME);
        ballBlockLeft = hardwareMap.servo.get(BALLBLOCKLEFTNAME);
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
        dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");

        gyroSensor.calibrate();

        while (gyroSensor.isCalibrating() )
        {
            telemetry.addData("Gyro", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Calibrated");
        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();


        leftButtonPusher.setPosition(0);
        rightButtonPusher.setPosition(0);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        colorSensorLeftBottom.enableLed(false);


        waitForStart();

        colorSensorLeftBottom.enableLed(true);

        move(130, .5);
        shoot();
        Left(45, 1);
        move(275, .5);
        Right(50, 1);
        lineSearch(.13);
        move(2, - .3);
        StrafetoWall(.5, 7);
        Beacon("Red");
        move(70, -.5);
        lineSearch(-.13);
        Beacon("Red");
        Strafe("turnRight", .5, 1);
        Right(90, 1);
        move(77, 1);
        Right(360, 1);
    }

    private double ticksPerRev = 7;
    private double gearBoxOne = 40.0;
    private double gearBoxTwo = 24.0 / 16.0;
    private double gearBoxThree = 1.0;
    private double wheelDiameter = 4.0 * Math.PI;
    private double cmPerInch = 2.54;

    private double width = 31.75;

    public double cmPerTick = (wheelDiameter / (ticksPerRev * gearBoxOne * gearBoxTwo * gearBoxThree)) * cmPerInch;


    public void initEncoders()  throws InterruptedException
    {
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        infeed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        infeed.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        infeed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setStrafePower(String Direction, double Speed)
    {
        if(Direction == "turnLeft")
        {
            rightFrontWheel.setPower(Speed);
            rightBackWheel.setPower(-Speed);
            leftFrontWheel.setPower(-Speed);
            leftBackWheel.setPower(Speed);
        }
        if(Direction == "turnRight")
        {
            rightFrontWheel.setPower(-Speed);
            rightBackWheel.setPower(Speed);
            leftFrontWheel.setPower(Speed);
            leftBackWheel.setPower(-Speed);
        }
    }


    public void move(double cm, double power) throws InterruptedException
    {
        initEncoders();

        Thread.sleep(50);

        double changeFactor = 0;
        double modified = cm + changeFactor;

        double ticks = modified / cmPerTick;

        setDrivePower(power);

        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());

        int Average = (RBPos + RFPos + LBPos + LFPos)/4;

        while(Average < ticks)
        {
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());

            Average = (RBPos + RFPos + LBPos + LFPos)/4;

            double currentDistance = Average * cmPerTick;

            telemetry.addData("Target Distance: ", cm);
            telemetry.addData("Current Distance: ", currentDistance);
            telemetry.update();
        }

        setDrivePower(0);

        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void Strafe(String Direction, double Speed, int Seconds) throws InterruptedException
    {
        initEncoders();

        int Time = Seconds*1000;
        setStrafePower(Direction, Speed);
        Thread.sleep(Time);
        setDrivePower(0);
    }

    public void Left(double degrees, double power) throws InterruptedException{

        initEncoders();
        double startTime = System.currentTimeMillis();

        double changeFactor = 90;
        double modified = changeFactor + degrees;

        double circleFrac = modified/360;
        double cm = width * circleFrac * Math.PI * 2;
        double movement = cm / cmPerTick;
        double ticks = movement/2;

        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
        leftBackWheel.setPower(-power);
        leftFrontWheel.setPower(-power);

        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());

        int Average = (RBPos + RFPos + LBPos + LFPos)/4;

        while(Average < ticks && opModeIsActive())
        {
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());

            Average = (RBPos + RFPos + LBPos + LFPos)/4;

            int currentHeading = getHeading();
            telemetry.addData("Target Heading: ", degrees);
            telemetry.addData("Current Heading: ", currentHeading);
            telemetry.update();
        }

        setDrivePower(0);

        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Right(double degrees, double power) throws InterruptedException {
        initEncoders();
        double startTime = System.currentTimeMillis();

        double changeFactor = 90;
        double modified = changeFactor + degrees;

        double circleFrac = modified/360;
        double cm = width * circleFrac * Math.PI * 2;
        double movement = cm / cmPerTick;
        double ticks = movement/2;

        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(-power);
        rightFrontWheel.setPower(-power);

        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());

        int Average = (RBPos + RFPos + LBPos + LFPos)/4;

        while(Average < ticks && opModeIsActive())
        {
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());

            Average = (RBPos + RFPos + LBPos + LFPos)/4;

            int currentHeading = getHeading();
            telemetry.addData("Target Heading: ", degrees);
            telemetry.addData("Current Heading: ", currentHeading);
            telemetry.update();
        }

        setDrivePower(0);

        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }


    public int getHeading() {

        int currentHeading = gyroSensor.getHeading();

        int output = currentHeading;

        if(currentHeading > 180 && currentHeading < 360) {
            output = currentHeading - 360;
        }

        return output;
    }

    public void checkDegrees(double target, double power) throws InterruptedException {
        double threshold = 0;

        while ((getHeading() <= target - threshold || getHeading() >= target + threshold) && opModeIsActive())
        {

            if (getHeading() <= target - threshold) {

                double degreeChange = Math.abs(getHeading()) - target;
                Right(degreeChange, power);

                telemetry.addData("Degree change", degreeChange);
                telemetry.addData("Current Heading", getHeading());
                telemetry.update();

            } else if (getHeading() >= target + threshold) {

                double degreeChange = getHeading() - target;
                Left(degreeChange, power);

                telemetry.addData("Degree change", degreeChange);
                telemetry.addData("Current Heading", getHeading());
                telemetry.update();
            }
        }
    }

    public void lineSearch(double Speed) throws InterruptedException {
        initEncoders();

        String color = "";
        telemetry.addLine("Line Searching ...");
        telemetry.update();
        setDrivePower(Speed);

        while(color == "" && opModeIsActive())
        {
            int red = colorSensorLeftBottom.red();
            int blue = colorSensorLeftBottom.blue();
            int alpha = colorSensorLeftBottom.alpha();

            if(red > 3 && blue > 3 && alpha > 3)
            {
                color = "White";
                setDrivePower(0);
                telemetry.addLine("Line Found!");
                telemetry.update();
            }

            telemetry.addData("Red: ", red);
            telemetry.addData("Blue: ", blue);
            telemetry.addData("Alpha: ", alpha);
            telemetry.update();
        }
        colorSensorLeftBottom.enableLed(false);

    }

    public void Beacon(String allianceColor) throws InterruptedException
    {
        String Color = "";
        telemetry.addLine("Detecting pressBeacon Color ...");

        while(Color == "" && opModeIsActive())
        {
            int red = colorSensorOnSide.red();
            int blue = colorSensorOnSide.blue();

            if (blue > 1)
            {
                Color = "Blue";
            }
            if (red > 1)
            {
                Color = "Red";
            }
        }

        telemetry.addData("Color Detected:", Color);
        telemetry.update();

        if(Color == allianceColor)
        {
            leftButtonPusher.setPosition(1);
            while(leftButtonPusher.getPosition() != 1 && opModeIsActive())
            {

            }
            leftButtonPusher.setPosition(0);
        }
        else
        {
            rightButtonPusher.setPosition(1);
            while(leftButtonPusher.getPosition() != 1 && opModeIsActive())
            {

            }
            rightButtonPusher.setPosition(0);
        }

    }

    public void shoot() throws InterruptedException
    {
        initEncoders();

        shoot1.setPower(1);
        shoot2.setPower(1);
        ballBlockRight.setPosition(0);
        ballBlockLeft.setPosition(1);
        Thread.sleep(700);
        infeed.setPower(1);
        Thread.sleep(400);
        infeed.setPower(0);
        Thread.sleep(1000);
        infeed.setPower(1);
        Thread.sleep(300);
        infeed.setPower(0);
        shoot1.setPower(0);
        shoot2.setPower(0);
    }

    public void StrafetoWall(double Speed, double Distance) throws InterruptedException
    {
        initEncoders();

        setStrafePower("turnLeft", Speed);

        while(range.getDistance(DistanceUnit.CM) > Distance && opModeIsActive())
        {
            telemetry.addData("Target Distance: ", Distance);
            telemetry.addData("Current Distance", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        setDrivePower(0);
    }

    public void setDrivePower(double power){
        leftFrontWheel.setPower(power);
        leftBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(power);

    }




}
