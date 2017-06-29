package org.firstinspires.ftc.teamcode.reference;

import android.os.Environment;

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
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Objects;
import java.util.Scanner;

/**
 * Created by Ethan Schaffer.
 */

@Autonomous(name="Autonomous", group="Autonomous")
@Disabled
public class AutoLinear extends LinearOpMode {
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
    //The Above Values lets us convert encoder ticks to centimeters per travelled, as shown below.

    public final double cmPerTick = (wheelDiameter / (ticksPerRev * gearBoxOne * gearBoxTwo * gearBoxThree)) * cmPerInch; //Allows us to drive our roobt with accuracy to the centiment


    //All of the different states the robot can be in during autonomous
    //These are our building blocks
    public enum states {
        Move, Shoot, EnableShot, ShootAtPower, TurnLeft, TurnRight, TurnLeftEnc, TurnRightEnc, StrafeLeft, StrafeRight, StrafeToWall, LineSearch, PressBeacon, WaitForTime, Finished, AlignToWithin
    }

    //The state object lets us declare any power or sensor values we want to.
    //Using this, we can change the power or sensor reading associated with any state
    //That allows us to have less states and more modularity
    public class state{
        public states sName = states.Finished;
        public double sensorValue = 0;
        public double powerValue = 0;
        public team alliance;
        state(states s, double firstV, double secondV){
            sName = s;
            sensorValue = firstV;
            powerValue = secondV;
        }
        state(states s, team c)
        {
            sName = s;
            alliance = c;
        }
        states getState(){
            return sName;
        }
        double getSensorValue(){
            return sensorValue;
        }
        double getPowerValue(){
            return powerValue;
        }
        team getAlliance(){
            return alliance;
        }


    }
    public class route{
        public state[] arr = {new state(states.Finished, 0, 0)};
        public String name = "Unnamed";
        route(String s, state[] a){
            name = s;
            arr = a;
        }
        String getName(){
            return name;
        }
        state [] getArr(){
            return arr;
        }
    }

    //Tolerance in degrees for turning
    //I don't think this actually works, but if it ain't broke...
    public final double tolerance = -5;

    //Editing these arrays would change how to auto runs, in it's entirety. If we could change these values from the phone, we could basically do doodle.
    public final state[] BlueShowOff = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,      - 1.00),
            new state(states.TurnRightEnc,   60,       0.35),
            new state(states.Move,          230,     - 1.00),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.StrafeToWall,  10,        0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.StrafeToWall,  8.5,       0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.LineSearch,    2,         0.10),
            new state(states.LineSearch,    2,       - 0.05),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.PressBeacon, team.Red         ),
            new state(states.StrafeRight,   5.00,      1.00),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.Move,          70,      - 1.00),
            new state(states.StrafeToWall,  8.5,       0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.LineSearch,    2,         0.05),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.Move,          1.5,     - 1.00),
            new state(states.LineSearch,    2,         0.10),
            new state(states.LineSearch,    2,       - 0.05),
            new state(states.PressBeacon, team.Red         ),
            new state(states.StrafeRight,   5.00,      1.00),
            new state(states.TurnRightEnc,  45,        1.00),
            new state(states.ShootAtPower,  0,         0.85),
            new state(states.Move,          145,       1.00),
            new state(states.EnableShot,    1500,      1.00),
            new state(states.ShootAtPower,  0,         0.00),
            new state(states.Move,          60,        1.00),
    };

    public final state[] BlueTwoBeaconParkArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,      - 1.00),
            new state(states.TurnRightEnc,   60,       0.35),
            new state(states.Move,          230,     - 1.00),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.StrafeToWall,  10,        0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.StrafeToWall,  8.5,       0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.LineSearch,    2,         0.10),
            new state(states.LineSearch,    2,       - 0.05),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.PressBeacon, team.Red         ),
            new state(states.StrafeRight,   5.00,      1.00),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.Move,          70,      - 1.00),
            new state(states.StrafeToWall,  8.5,       0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.LineSearch,    2,         0.05),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.Move,          1.5,     - 1.00),
            new state(states.LineSearch,    2,         0.10),
            new state(states.LineSearch,    2,       - 0.05),
            new state(states.PressBeacon, team.Red         ),
            new state(states.StrafeRight,   5.00,      1.00),
            new state(states.TurnRightEnc,  45,        1.00),
            new state(states.Move,          205,       1.00),
    };
    public final state[] BlueOneBeaconShootWood = new state[]{
            new state(states.Move,          60,      - 1.00),
            new state(states.TurnRightEnc,   60,       0.35),
            new state(states.Move,          230,     - 1.00),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.StrafeToWall,  10,        0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.StrafeToWall,  8.5,       0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.LineSearch,    2,         0.10),
            new state(states.LineSearch,    2,       - 0.05),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.PressBeacon, team.Red         ),
            new state(states.ShootAtPower,  0,         0.85),
            new state(states.StrafeRight,   1.0,       1.00), //Away from wall
            new state(states.TurnRightEnc,  180,       1.00),
            new state(states.Move,          90,        1.00),
            new state(states.EnableShot,    1500,      1.00),
            new state(states.ShootAtPower,  0,         0.00),
            new state(states.Move,          60,        1.00),
    };
    public final state[] BlueTwoBeaconsArray = new state[] {
            new state(states.Move,          60,      - 1.00),
            new state(states.TurnRightEnc,   60,       0.35),
            new state(states.Move,          230,     - 1.00),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.StrafeToWall,  10,        0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.StrafeToWall,  8.5,       0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.LineSearch,    2,         0.10),
            new state(states.LineSearch,    2,       - 0.05),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.PressBeacon, team.Red         ),
            new state(states.StrafeRight,   5.00,      1.00),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.Move,          70,      - 1.00),
            new state(states.StrafeToWall,  8.5,       0.10),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.LineSearch,    2,         0.05),
            new state(states.AlignToWithin, tolerance, 0.05),
            new state(states.Move,          1.5,    - 1.00),
            new state(states.LineSearch,    2,        0.10),
            new state(states.LineSearch,    2,      - 0.05),
            new state(states.PressBeacon, team.Red       ),
    };

    public final state[] RedShowOff = new state[]{
            //              State         Sensor       Power
            new state(states.ShootAtPower,      0,     0.85),
            new state(states.Move,            105,     1.00),
            new state(states.EnableShot,     1500,     1.00),
            new state(states.ShootAtPower,      0,     0.00),
            new state(states.Move,          45,     - 1.00),
            new state(states.TurnLeftEnc,   60,       0.35),
            new state(states.Move,          230,      1.00),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.StrafeToWall,  10,       0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.StrafeToWall,  8.5,      0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.LineSearch,    2,      - 0.10),
            new state(states.LineSearch,    2,        0.05),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.PressBeacon, team.Red        ),
            new state(states.StrafeRight,   5.00,     1.00),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.Move,          70,       1.00),
            new state(states.StrafeToWall,  8.5,      0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.LineSearch,    2,        0.10),
            new state(states.LineSearch,    2,      - 0.05),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.Move,          1.5,      1.00),
            new state(states.LineSearch,    2,      - 0.10),
            new state(states.LineSearch,    2,        0.05),
            new state(states.PressBeacon, team.Red        ),
            new state(states.StrafeRight,   5.00,     1.00),
            new state(states.TurnLeftEnc,   45,       1.00),
            new state(states.Move,          205,    - 1.00),
    };

    public final state[] RedTwoBeaconParkArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,       1.00),
            new state(states.TurnLeftEnc,   60,       0.35),
            new state(states.Move,          230,      1.00),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.StrafeToWall,  10,       0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.StrafeToWall,  8.5,      0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.LineSearch,    2,      - 0.10),
            new state(states.LineSearch,    2,        0.05),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.PressBeacon, team.Red        ),
            new state(states.StrafeRight,   5.00,     1.00),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.Move,          70,       1.00),
            new state(states.StrafeToWall,  8.5,      0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.LineSearch,    2,        0.10),
            new state(states.LineSearch,    2,      - 0.05),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.Move,          1.5,      1.00),
            new state(states.LineSearch,    2,      - 0.10),
            new state(states.LineSearch,    2,        0.05),
            new state(states.PressBeacon, team.Red        ),
            new state(states.StrafeRight,   5.00,     1.00),
            new state(states.TurnLeftEnc,   45,       1.00),
            new state(states.Move,          205,    - 1.00),
    };
    public final state[] RedOneBeaconShootWood = new state[]{
            new state(states.Move,          60,       1.00),
            new state(states.TurnLeftEnc,   60,       0.35),
            new state(states.Move,          230,      1.00),
            new state(states.TurnRight,     15,       0.25),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.StrafeToWall,  10,       0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.StrafeToWall,  8.5,      0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.LineSearch,    2,      - 0.10),
            new state(states.LineSearch,    2,        0.05),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.PressBeacon, team.Red        ),
            new state(states.ShootAtPower,  0,        0.85),
            new state(states.StrafeRight,   1.0,      1.00), //Away from wall
            new state(states.TurnRightEnc,  180,      1.00),
            new state(states.Move,          90,       1.00),
            new state(states.EnableShot,    1500,     1.00),
            new state(states.ShootAtPower,  0,        0.00),
            new state(states.Move,          60,       1.00),
    };
    public final state[] RedTwoBeaconsArray = new state[] {
            new state(states.Move,          60,       1.00),
            new state(states.TurnLeftEnc,   60,       0.35),
            new state(states.Move,          230,      1.00),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.StrafeToWall,  10,       0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.StrafeToWall,  8.5,      0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.LineSearch,    2,      - 0.10),
            new state(states.LineSearch,    2,        0.05),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.PressBeacon, team.Red        ),
            new state(states.StrafeRight,   5.00,     1.00),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.Move,          70,       1.00),
            new state(states.StrafeToWall,  8.5,      0.10),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.LineSearch,    2,        0.10),
            new state(states.LineSearch,    2,      - 0.05),
            new state(states.AlignToWithin, tolerance,0.05),
            new state(states.Move,          1.5,      1.00),
            new state(states.LineSearch,    2,      - 0.10),
            new state(states.LineSearch,    2,        0.05),
            new state(states.PressBeacon, team.Red        ),
    };

    public final state[] GenericShootFromFarAngleArray = new state[] {
            new state(states.Move,     180,      .75),
            new state(states.Shoot,      0,        0),
            new state(states.Move,      60,       .5),
            new state(states.Move,      60,     - .5),
            new state(states.Move,      60,      1.0),
    };
    public final state[] GenericShootFromNextToRampArray = new state[] {
            new state(states.Move,     105,      .75),
            new state(states.Shoot,      0,        0),
            new state(states.Move,      60,       .5),
            new state(states.Move,      60,     - .5),
            new state(states.Move,      60,      1.0),
    };

    public final state[] DoNothingArray = new state[] {
            new state(states.Finished,0,0)
        };
    route [] RouteArray = {
            new route("Blue Show Off", BlueShowOff),
            new route("Blue 1 Beacon Shoot", BlueOneBeaconShootWood),
            new route("Blue 2 Beacon Park", BlueTwoBeaconParkArray),
            new route("Blue 2 Beacon", BlueTwoBeaconsArray),
            new route("Red Show Off", RedShowOff),
            new route("Red 1 Beacon Shoot", RedOneBeaconShootWood),
            new route("Red 2 Beacon Park", RedTwoBeaconParkArray),
            new route("Red 2 Beacon", RedTwoBeaconsArray),
            new route("Shoot Far Angle", GenericShootFromFarAngleArray),
            new route("Shoot Near", GenericShootFromNextToRampArray),
    };

    public state[] stateOrder;
    //NotSensed is for the Color Sensor while we are pushing the beacon.
    public enum team {
        Red, Blue, NotSensed
    }

    //Declaration of the Robot iself
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
    public state CurrentState;
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

    public int stateNumber = -1;
    public state updateState() throws InterruptedException{
        //By Resetting encoders and other values (like time buffers) here, we avoid having to do more advanced logic within each case statement
        setDrivePower(0);
        idle();
        stateNumber++;
        colorReading = team.NotSensed;
        initialized = false;
        movedServo = false;
        time = 0;
        idle();
        if(stateNumber == stateOrder.length){
            return new state(states.Finished, 0, 0);
        }
        return stateOrder[stateNumber];
    }

    public state getCurrentState(){
        return stateOrder[stateNumber];
    } //We never use this, but it could be useful one day.

    @Override
    public void runOpMode() throws InterruptedException {
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRONAME);
        gyroSensor.calibrate(); //Calibrating Gyro is important! It not only zeroes it, but it also
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
/*
        I2cDevice navXPortGetter = hardwareMap.get(I2cDevice.class, "n");
        navX = new AHRS(dim, navXPortGetter.getPort(), AHRS.DeviceDataType.kProcessedData, (byte)50);
*/
        int timer = 0;
        int cycler = 0;

        //Based on the value of autoRouteChosen, we set the stateOrder array to the correct value
        String autoRouteChosen = "RedBeaconParkRamp";
        int pos = 3;
        String[] routes = {
                "GenericShootFromNextToRamp",
                "RedOneBeaconShootWood",
                "RedTwoBeacons",
                "RedTwoBeaconPark",  //position starts here
                "BlueTwoBeaconPark",
                "BlueOneBeaconShootWood",
                "BlueTwoBeacons",
                "GenericShootFromFarAngle",
        };
        boolean prevX = false, prevB = false;
        while (!gamepad1.y) {
            if (gamepad1.x && !prevX) {
                pos--;
            } else if (gamepad1.b && !prevB) {
                pos++;
            }
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            if (pos < 0) {
                pos = RouteArray.length - 1;
            }
            if (pos == RouteArray.length) {
                pos = 0;
            }
            autoRouteChosen = RouteArray[pos].getName();
            telemetry.addData("Route", autoRouteChosen);
            telemetry.update();
        }
        stateOrder = RouteArray[pos].getArr();
/*
        stateOrder = DoNothingArray;
        switch (autoRouteChosen) {
            case "RedOneBeaconShootWood":
                stateOrder = RedOneBeaconShootWood;
                break;
            case "RedTwoBeacons":
                stateOrder = RedTwoBeaconsArray;
                break;
            case "RedTwoBeaconPark":
                stateOrder = RedTwoBeaconParkArray;
                break;
            case "BlueOneBeaconShootWood":
                stateOrder = BlueOneBeaconShootWood;
                break;
            case "BlueTwoBeacons":
                stateOrder = BlueTwoBeaconsArray;
                break;
            case "BlueTwoBeaconPark":
                stateOrder = BlueTwoBeaconParkArray;
                break;
            case "GenericShootFromFarAngle":
                stateOrder = GenericShootFromFarAngleArray;
                break;
            case "GenericShootFromNextToRamp":
                stateOrder = GenericShootFromNextToRampArray;
                break;
            //Currently, there are 10 choices for autonomous routes: 4 per alliance color, and two reserve options
            //We can also use ReadInput to have an infinite number of finely tweaked routes
            case "ReadInput":
                String FILE_DIR = "/InputFiles/";
                File file = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + FILE_DIR, "input_no_color");
                Scanner s = null;
                try {
                    s = new Scanner(file);
                } catch (FileNotFoundException e) {
                    stateOrder = DoNothingArray;
                    e.printStackTrace();
                    break;
                }
                try {
                    state[] inputArray = new state[100]; //make the array large enough to never have issues
                    int positionInArrayAsWeAdd = 0;
                    while (s.hasNextDouble()) {
                        double stateVal;
                        double powerVal;
                        double sensorVal;
                        stateVal = s.nextDouble();
                        powerVal = s.nextDouble();
                        sensorVal = s.nextDouble();
                        switch ((int) stateVal) {
                            case 1:
                                inputArray[positionInArrayAsWeAdd] = new state(states.Move, sensorVal, powerVal);
                                break;
                            case 2:
                                inputArray[positionInArrayAsWeAdd] = new state(states.Shoot, sensorVal, powerVal);
                                break;
                            case 3:
                                inputArray[positionInArrayAsWeAdd] = new state(states.EnableShot, sensorVal, powerVal);
                                break;
                            case 4:
                                inputArray[positionInArrayAsWeAdd] = new state(states.ShootAtPower, sensorVal, powerVal);
                                break;

                            case 5:
                                inputArray[positionInArrayAsWeAdd] = new state(states.TurnLeft, sensorVal, powerVal);
                                break;
                            case 6:
                                inputArray[positionInArrayAsWeAdd] = new state(states.TurnRight, sensorVal, powerVal);
                                break;
                            case 7:
                                inputArray[positionInArrayAsWeAdd] = new state(states.TurnLeftEnc, sensorVal, powerVal);
                                break;
                            case 8:
                                inputArray[positionInArrayAsWeAdd] = new state(states.TurnRightEnc, sensorVal, powerVal);
                                break;
                            case 9:
                                inputArray[positionInArrayAsWeAdd] = new state(states.StrafeLeft, sensorVal, powerVal);
                                break;
                            case 10:
                                inputArray[positionInArrayAsWeAdd] = new state(states.StrafeRight, sensorVal, powerVal);
                                break;
                            case 11:
                                inputArray[positionInArrayAsWeAdd] = new state(states.StrafeToWall, sensorVal, powerVal);
                                break;
                            case 12:
                                inputArray[positionInArrayAsWeAdd] = new state(states.LineSearch, sensorVal, powerVal);
                                break;
                            case 13:
                                inputArray[positionInArrayAsWeAdd] = new state(states.PressBeacon, sensorVal, powerVal);
                                break;
                            case 14:
                                inputArray[positionInArrayAsWeAdd] = new state(states.WaitForTime, sensorVal, powerVal);
                                break;
                            case 15:
                                inputArray[positionInArrayAsWeAdd] = new state(states.Finished, sensorVal, powerVal);
                                break;
                            case 16:
                                inputArray[positionInArrayAsWeAdd] = new state(states.AlignToWithin, sensorVal, powerVal);
                                break;
                        }
                        positionInArrayAsWeAdd++;
                    }
                    inputArray[positionInArrayAsWeAdd + 1] = new state(states.Finished, 0, 0); //automatically ends the autonomoous after all commands have been tried
                    stateOrder = inputArray;
                } catch (Exception e) {
                    e.printStackTrace();
                    telemetry.clear();
                    telemetry.addLine("Something went wrong within the building of the array");
                    telemetry.update();
                    stateOrder = DoNothingArray;
                }
                break;
            case "DoNothing":
                stateOrder = DoNothingArray;
                break;
            default:
                telemetry.addData("Huh", "This is strange");
                telemetry.update();
                stateOrder = DoNothingArray;
                break;
        }
*/
        timer = 0;
/*
        while (gyroSensor.isCalibrating()) {
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
        telemetry.addData("Route", Objects.equals(autoRouteChosen, "ReadInput") ? "Custom" : autoRouteChosen);
*/
        if (range.rawUltrasonic() > 0 && colorSensorOnSide.alpha() == 0 && colorSensorLeftBottom.alpha() > 0 && timer < 10000) {
            //Range Works               //Side not sensing (no light)     //Bottom senses something           //Gyro set up in a reasonable time frame
            telemetry.addLine("Everything is good to go!");
        } else {
            telemetry.addData("Sensors", "Something Wrong");
        }
        telemetry.update();
        CurrentState = updateState(); // Set Current State
        colorSensorLeftBottom.enableLed(true); //If you don't set the LED until after the waitForStart(), it doesn't work.
        colorSensorOnSide.enableLed(false);
        double cm, ticks, degrees, power, Seconds, Speed, Time;
        int RBPos, RFPos, LBPos, LFPos, Average;


        waitForStart();

        for (int i = 0; i < stateOrder.length; i++) {
            CurrentState = stateOrder[i];
            ResetDriveEncoders();
            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
            if(CurrentState.getState() == states.Move){
                Move(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.EnableShot){
                EnableShot(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.LineSearch){
                LineSearch(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.PressBeacon){
                PressBeacon(CurrentState.getAlliance());
            } else if(CurrentState.getState() == states.ShootAtPower){
                ShootAtPower(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.StrafeLeft){
                StrafeLeft(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.StrafeRight){
                StrafeRight(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.TurnLeft){
                TurnLeft(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.TurnLeftEnc){
                TurnLeftEnc(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.TurnRight){
                TurnRight(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.TurnRightEnc){
                TurnRightEnc(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.StrafeToWall){
                StrafeToWall(CurrentState.getSensorValue(), CurrentState.getPowerValue());
            } else if(CurrentState.getState() == states.WaitForTime) {
                sleep((long) CurrentState.getSensorValue());
            } else if(CurrentState.getState() == states.Shoot) {
                ShootAtPower(0, 1.0);
                EnableShot(1500, 1.0);
                ShootAtPower(0, 0.0);
            } else if(CurrentState.getState() == states.AlignToWithin) {
                TurnRight(-CurrentState.getSensorValue(), CurrentState.getPowerValue());
                if(CurrentState.getSensorValue() > .04){
                    TurnLeft(CurrentState.getSensorValue(), CurrentState.getPowerValue()-.01);
                } else {
                    TurnLeft(CurrentState.getSensorValue(), CurrentState.getPowerValue());
                }
            } else {
                super.stop();
            }
        }
    }
        public void Move(double sensor, double power) {
            double ticks = sensor / cmPerTick;
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            double avg = (RBPos + LBPos + RFPos + LFPos)/4;
            while(avg < ticks) {
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
            while(gyroSensor.getIntegratedZValue() <= sensor){
                rightBackWheel.setPower(power);
                rightFrontWheel.setPower(power);
                leftBackWheel.setPower(-power);
                leftFrontWheel.setPower(-power);
            }
            setDrivePower(0);

        }
        public void TurnRight(double sensor, double power){
            while(gyroSensor.getIntegratedZValue() >= sensor){
                rightBackWheel.setPower(-power);
                rightFrontWheel.setPower(-power);
                leftBackWheel.setPower(power);
                leftFrontWheel.setPower(power);
            }
            setDrivePower(0);
        }
        public void TurnLeftEnc(double sensor, double power){
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
            } while(avg < ticks);
            setDrivePower(0);
        }
        public void TurnRightEnc(double sensor, double power){
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
            } while(avg < ticks);
            setDrivePower(0);
        }
    public void StrafeLeft(double sensor, double power){
        double ticks = sensor / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos)/4;
        while(avg < ticks) {
            setStrafePower("Left", power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        setDrivePower(0);
    }
        public void StrafeRight(double sensor, double power){
            double ticks = sensor / cmPerTick;
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            double avg = (RBPos + LBPos + RFPos + LFPos)/4;
            while(avg < ticks) {
                setStrafePower("Right", power);
                RBPos = Math.abs(rightBackWheel.getCurrentPosition());
                RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
                LBPos = Math.abs(leftBackWheel.getCurrentPosition());
                LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
                avg = (RBPos + LBPos + RFPos + LFPos) / 4;
            }
            setDrivePower(0);
        }
        public void StrafeToWall(double sensor, double power){
            if(sensor != (int) sensor){
                sensor++;
            }
            while(range.getDistance(DistanceUnit.CM) > sensor){
                setStrafePower("Left", power);
            }
            if(sensor != (int) sensor){ // if is half input
                StrafeLeft(.025, power);
            }
            setDrivePower(0);
        }
        public void LineSearch(double sensor, double power){
            while(colorSensorLeftBottom.red() < sensor && colorSensorLeftBottom.blue() < sensor && colorSensorLeftBottom.alpha() < sensor){
                setDrivePower(power);
            }
            setDrivePower(0);
        }
        public void PressBeacon(team t){
            team colorReading;
            if(colorSensorOnSide.red() > colorSensorOnSide.blue()){
                colorReading = team.Red;
            } else {
                colorReading = team.Blue;
            }
            if(colorReading == t){
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
            shoot1.setPower(power);
            shoot2.setPower(power);
        }
        public void EnableShot(double sensor, double power){
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
