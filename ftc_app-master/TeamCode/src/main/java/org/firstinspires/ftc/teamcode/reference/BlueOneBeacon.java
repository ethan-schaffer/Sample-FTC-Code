package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

import java.util.Objects;

/**
 * Created by Ethan Schaffer.
 */

@Autonomous(name="B One Beacon", group="Blue")
@Disabled
public class BlueOneBeacon extends LinearOpMode {
    public void setDrivePower(double power) {
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
    }

    public void ResetDriveEncoders(){
        leftBackWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(leftFrontWheel.getCurrentPosition() != 0 || leftBackWheel.getCurrentPosition() != 0
                || rightBackWheel.getCurrentPosition() != 0 || rightFrontWheel.getCurrentPosition() != 0){

        }
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setStrafePower(String Direction, double Speed) {
        if(Objects.equals(Direction, "Left"))
        {
            rightFrontWheel.setPower(Speed);
            rightBackWheel.setPower(-Speed);
            leftFrontWheel.setPower(-Speed);
            leftBackWheel.setPower(Speed);
        }
        if(Objects.equals(Direction, "Right"))
        {
            rightFrontWheel.setPower(-Speed);
            rightBackWheel.setPower(Speed);
            leftFrontWheel.setPower(Speed);
            leftBackWheel.setPower(-Speed);
        }
    }

    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String BALLBLOCKLEFTNAME = "bl";
    public static final String BALLBLOCKRIGHTNAME = "br"; //MO Ports 3+4
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String GYRONAME = "g"; //Port 4
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 2
    public static final String COLORLEFTBOTTOMNAME = "cb";//Port 3
    public static final String COLORRIGHTBOTTOMNAME = "cb2"; //Port 4

    public static final double LEFT_SERVO_OFF_VALUE = .20;
    public static final double LEFT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_OFF_VALUE = .20;
    public static final double BALLBLOCKLEFTOPEN = 1;
    public static final double BALLBLOCKLEFTCLOSED = 0;
    public static final double BALLBLOCKRIGHTOPEN = 0;
    public static final double BALLBLOCKRIGHTCLOSED = 1;

    public final double ticksPerRev = 7;
    public final double gearBoxOne = 40.0;
    public final double gearBoxTwo = 24.0 / 16.0;
    public final double gearBoxThree = 1.0;
    public final double wheelDiameter = 4.0 * Math.PI;
    public final double cmPerInch = 2.54;
    public final double width = 31.75;
    public final double ticksToStrafeDistance = 2000/(172*cmPerInch);
    //The Above Values lets us convert encoder ticks to centimeters per travelled, as shown below.

    public final double cmPerTick = (wheelDiameter / (ticksPerRev * gearBoxOne * gearBoxTwo * gearBoxThree)) * cmPerInch; //Allows us to drive our roobt with accuracy to the centiment
    //NotSensed is for the Color Sensor while we are pushing the beacon.
    public enum team {
        Red, Blue, NotSensed
    }

    //Declaration of the Robot itself
    public DcMotor leftFrontWheel;
    public DcMotor leftBackWheel;
    public DcMotor rightFrontWheel;
    public DcMotor rightBackWheel;
    public DcMotor shoot1;
    public DcMotor shoot2;
    public DcMotor infeed;
    public Servo leftButtonPusher;
    public Servo rightButtonPusher;
    public Servo ballBlockRight;
    public Servo ballBlockLeft;
    public ColorSensor colorSensorLeftBottom;
    public ColorSensor colorSensorOnSide;
    public ModernRoboticsI2cRangeSensor range;
    public ModernRoboticsI2cGyro gyroSensor;
    public DeviceInterfaceModule dim;
    public AHRS navX;

    //Useful variables
    public long lastTime;
    public long time;
    public boolean initialized = false;
    public boolean movedServo = false;
    public double circleFrac;
    public double movement;
    public double changeFactor;
    public double modified;
    public double currentDistance;
    public team colorReading = team.NotSensed;
    public int redReading;
    public int blueReading;
    //By declaring certain values here and not within our case statements, we avoid declaring them multiple times.

    @Override
    public void runOpMode() throws InterruptedException {
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

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        colorSensorLeftBottom = hardwareMap.colorSensor.get(COLORLEFTBOTTOMNAME);
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorLeftBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");

        //Based on the value of autoRouteChosen, we set the stateOrder array to the correct value
        navX = new AHRS(dim, 5, AHRS.DeviceDataType.kProcessedData, (byte)50);
        navX.zeroYaw();
        int cycler = 0;
        int timer = 0;

        while (navX.isCalibrating()) {
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
        telemetry.addData("Yaw", navX.getYaw());
        if(!navX.isConnected()){
            telemetry.addData("NavX", "DISCONNECTED!");
        } else {
            telemetry.addData("NavX", "Connected!");}
        telemetry.update();

        colorSensorLeftBottom.enableLed(true); //If you don't set the LED until after the waitForStart(); it doesn't work.
        colorSensorOnSide.enableLed(false);

        waitForStart();

        Move(80, - 1.00); //Move to turn position
        TurnRight(30, 0.10); //Turn to face Left Wall
        Move(225,- 1.00); //Move to left wall
        AlignToWithin(3, 0.05); //Line up
        StrafeToWall(23, 0.20); //Strafe to the wall
        AlignToWithin(3, 0.05); //Line up
        LineSearch(2, 0.10); //Backwards to line
        StrafeToWall(10, 0.10); //Get to the right
        Move(5, 1.00); //Move to left wall
        LineSearch(2, - 0.10); //Forwards to line
        LineSearch(2, 0.05); //Backwards to line again, slower to get better accuracy
        StrafeToWall(8, 0.10); //Get to the right
        PressBeacon(team.Blue ); //Press red button

    }
    public void AlignToWithin(double sensor, double power){
        TurnRight( - sensor, power);
        TurnLeft(sensor, power);
        TurnRight( - sensor, power);
    }
    public void Move(double sensor, double power) {
        if(!opModeIsActive())
            super.stop();
        ResetDriveEncoders();
        double ticks = sensor / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos)/4;
        while(avg < ticks && opModeIsActive()) {
            setDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        setDrivePower(0);
    }

    public void TurnLeft(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        while(navX.getYaw() >= sensor && opModeIsActive()){
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        setDrivePower(0);

    }
    public void TurnRight(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        while(navX.getYaw() <= sensor && opModeIsActive()){
            rightBackWheel.setPower(-power);
            rightFrontWheel.setPower(-power);
            leftBackWheel.setPower(power);
            leftFrontWheel.setPower(power);
        }
        setDrivePower(0);
    }

    public void TurnLeftEnc(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        ResetDriveEncoders();
        changeFactor = 90;
        modified = changeFactor + sensor;

        circleFrac = modified/360;
        double cm = width * circleFrac * Math.PI * 2;
        movement = cm / cmPerTick;
        double ticks = movement/2;

        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
        leftBackWheel.setPower(-power);
        leftFrontWheel.setPower(-power);

        double avg = 0;
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos)/4;
        } while(avg < ticks && opModeIsActive());
        setDrivePower(0);
    }
    public void TurnRightEnc(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        ResetDriveEncoders();
        changeFactor = 90;
        modified = changeFactor + sensor;

        circleFrac = modified/360;
        double cm = width * circleFrac * Math.PI * 2;
        movement = cm / cmPerTick;
        double ticks = movement/2;

        rightBackWheel.setPower(-power);
        rightFrontWheel.setPower(-power);
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);

        double avg = 0;
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos)/4;
        } while(avg < ticks && opModeIsActive());
        setDrivePower(0);
    }
    public void StrafeLeft(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        double ticks = sensor / cmPerTick;
        ticks *= ticksToStrafeDistance;
        int avg = 0;
        setStrafePower("Left", power);
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos)/4;
        } while(avg < ticks && opModeIsActive());
        setDrivePower(0);
    }
    public void StrafeRight(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        double ticks = sensor / cmPerTick;
        ticks *= ticksToStrafeDistance;
        int avg = 0;
        setStrafePower("Right", power);
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos)/4;
        } while(avg < ticks && opModeIsActive());
        setDrivePower(0);
    }
    public double getRange(double previous){
        double c = range.getDistance(DistanceUnit.CM);
        if(c == 255){
            return previous;
        } else {
            return c;
        }
    }
    public void StrafeToWall(double sensor, double power){
        double pastRange = 254;
        if(!opModeIsActive())
            super.stop();
        while(pastRange > sensor && opModeIsActive()){
            pastRange = getRange(pastRange);
            setStrafePower("Left", power);
            telemetry.addData("Distance", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Light", range.getLightDetected());
            telemetry.update();
        }
        if(range.getDistance(DistanceUnit.CM) == 255){
            StrafeToWall(sensor, power);
        }
        setDrivePower(0);
    }
    public void StrafeFromWall(double sensor, double power){
        double pastRange = 1;
        if(!opModeIsActive())
            super.stop();
        while(pastRange < sensor && opModeIsActive()){
            pastRange = getRange(pastRange);
            setStrafePower("Left", power);
            telemetry.addData("Distance", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Light", range.getLightDetected());
            telemetry.update();
        }
        if(range.getDistance(DistanceUnit.CM) == 255){
            StrafeToWall(sensor, power);
        }
        setDrivePower(0);
    }

    public void LineSearch(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        while(colorSensorLeftBottom.red() < sensor && colorSensorLeftBottom.blue() < sensor && colorSensorLeftBottom.alpha() < sensor && opModeIsActive()){
            setDrivePower(power);
        }
        setDrivePower(0);
    }
    public void PressBeacon(team t){
        if(!opModeIsActive())
            super.stop();
        team colorReading;
        if(colorSensorOnSide.red() > colorSensorOnSide.blue()){
            colorReading = team.Red;
        } else {
            colorReading = team.Blue;
        }
        if(colorReading == t){
            Move(1, - .25);
            rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        } else {
            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
        }
        try {
            Thread.sleep(1250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void ShootAtPower(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        shoot1.setPower(power);
        shoot2.setPower(power);
    }
    public void EnableShot(double sensor, double power){
        if(!opModeIsActive())
            super.stop();
        ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
        ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
        infeed.setPower(power);
        try{
            Thread.sleep((long)sensor);
        } catch (InterruptedException e){
            e.printStackTrace();
        }
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        infeed.setPower(0);
    }
}
