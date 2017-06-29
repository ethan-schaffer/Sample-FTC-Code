package org.firstinspires.ftc.teamcode.reference;

import android.os.Environment;
import android.os.SystemClock;

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
import org.firstinspires.ftc.teamcode.options.OptionMenu;
import org.firstinspires.ftc.teamcode.options.SingleSelectCategory;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.Objects;
import java.util.Scanner;

/**
 * Created by Ethan Schaffer.
 */

@Autonomous(name="Autonomous State", group="Autonomous")
@Disabled
public class Overall_Autonomous extends LinearOpMode {
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
        Move, Shoot, EnableShot, ShootAtPower, TurnLeft, TurnRight, TurnLeftEnc, TurnRightEnc, StrafeLeft, StrafeRight, StrafeToWall, LineSearch, PressBeacon, WaitForTime, Finished
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

    //Tolerance in degrees for turning
    //I don't think this actually works, but if it ain't broke...
    public final double tolerance = -5;

    //Editing these arrays would change how to auto runs, in it's entirety. If we could change these values from the phone, we could basically do doodle.
    public final state[] BlueBeaconParkWoodArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,     - 1.00),
            new state(states.TurnRightEnc,   60,       0.25),
            new state(states.Move,          230,    - 1.00),
            new state(states.TurnLeft,   - 25,       0.15),
            new state(states.StrafeToWall,  13,       0.13),
            new state(states.TurnLeft,      tolerance,        0.05),
            new state(states.TurnRight,     -tolerance,       0.05),
            new state(states.LineSearch,    2,         0.10),
            new state(states.StrafeToWall,  8,         0.10),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.PressBeacon, team.Blue       ),
            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall
            new state(states.Move,          125,     - 0.50),
            new state(states.TurnRight,    -tolerance,         0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,         0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.StrafeToWall,  11,         0.13),
            new state(states.LineSearch,    2,        0.10),
            new state(states.StrafeToWall,  8,         0.10),
            new state(states.PressBeacon, team.Blue       ),
            new state(states.StrafeRight,   0.25,      1.00), //Away from wall
            new state(states.TurnRightEnc,   45,       1.00),
            new state(states.Move,          225,        1.00),
    };
    public final state[] BlueBeaconShootParkWoodArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,     - 1.00),
            new state(states.TurnRightEnc,   60,       0.25),
            new state(states.Move,          230,    - 1.00),
            new state(states.TurnLeft,   - 25,       0.15),
            new state(states.StrafeToWall,  13,       0.13),
            new state(states.TurnLeft,      tolerance,        0.05),
            new state(states.TurnRight,     -tolerance,       0.05),
            new state(states.LineSearch,    2,         0.10),
            new state(states.StrafeToWall,  8,         0.10),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.PressBeacon, team.Blue       ),
            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall
            new state(states.Move,          125,     - 0.50),
            new state(states.TurnRight,    -tolerance,         0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,         0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.StrafeToWall,  11,         0.13),
            new state(states.LineSearch,    2,        0.10),
            new state(states.StrafeToWall,  8,         0.10),
            new state(states.PressBeacon, team.Blue       ),
            new state(states.StrafeRight,   0.25,      1.00), //Away from wall
            new state(states.TurnRightEnc,   45,       1.00),
            new state(states.ShootAtPower,  0,       1.00),
            new state(states.Move,          155,        1.00),
            new state(states.EnableShot,    1500,    1.00),
            new state(states.ShootAtPower,  0,       0.00),

            new state(states.Move,          60,        1.00),
    };
    public final state[] BlueBeaconShootParkRampArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          105,      - 1.00),
            new state(states.TurnRightEnc,   45,        0.25),
            new state(states.Move,          145,      - 1.00),
            new state(states.TurnLeft,   -   25,        0.15),
            new state(states.StrafeToWall,   13,        0.13),
            new state(states.TurnLeft,      tolerance,  0.05),
            new state(states.TurnRight,     -tolerance, 0.05),
            new state(states.LineSearch,    2,        - 0.10),
            new state(states.StrafeToWall,  8,          0.10),
            new state(states.LineSearch,    2,          0.10),
            new state(states.PressBeacon, team.Blue       ),
            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall
            new state(states.Move,          125,        0.50),
            new state(states.TurnRight,    -tolerance,  0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,   0.05),
            new state(states.LineSearch,    2,          0.10),
            new state(states.StrafeToWall,  11,         0.13),
            new state(states.LineSearch,    2,        - 0.10),
            new state(states.StrafeToWall,  8,          0.10),
            new state(states.PressBeacon, team.Blue       ),
            new state(states.ShootAtPower, 0,            1.00),
            new state(states.StrafeRight,   0.25,       1.00), //Away from wall
            new state(states.TurnLeftEnc,   90,         1.00),
            new state(states.EnableShot,    1500,     1.00),
            new state(states.ShootAtPower,  0,       0.00),
            new state(states.TurnRightEnc,   45,         1.00),
            new state(states.Move,          105,      - 1.00),
    };
    public final state[] BlueBeaconParkRampArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          105,      - 1.00),
            new state(states.TurnRightEnc,   45,        0.25),
            new state(states.Move,          145,      - 1.00),
            new state(states.TurnLeft,   -   25,        0.15),
            new state(states.StrafeToWall,   13,        0.13),
            new state(states.TurnLeft,      tolerance,  0.05),
            new state(states.TurnRight,     -tolerance, 0.05),
            new state(states.LineSearch,    2,        - 0.10),
            new state(states.StrafeToWall,  8,          0.10),
            new state(states.LineSearch,    2,          0.10),
            new state(states.PressBeacon, team.Blue       ),
            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall
            new state(states.Move,          125,        0.50),
            new state(states.TurnRight,    -tolerance,  0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,   0.05),
            new state(states.LineSearch,    2,          0.10),
            new state(states.StrafeToWall,  11,         0.13),
            new state(states.LineSearch,    2,        - 0.10),
            new state(states.StrafeToWall,  8,          0.10),
            new state(states.PressBeacon, team.Blue       ),
            new state(states.StrafeRight,   0.25,       1.00), //Away from wall
            new state(states.TurnLeftEnc,   45,         1.00),
    };

    public final state[] RedBeaconParkWoodArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,       1.00),
            new state(states.TurnLeftEnc,   60,       0.25),
            new state(states.Move,          230,    1.00),
            new state(states.TurnRight,     25,       0.15),
            new state(states.StrafeToWall,  11,         0.10),
            new state(states.TurnRight,     -tolerance,       0.05),
            new state(states.TurnLeft,      tolerance,        0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.StrafeToWall,  9,         0.10),
            new state(states.PressBeacon, team.Red       ),
            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall
            new state(states.Move,          125,       0.50),
            new state(states.TurnRight,    -tolerance,         0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,         0.05),
            new state(states.LineSearch,    2,         0.10),
            new state(states.StrafeToWall,  11,         0.13),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.StrafeToWall,  8,         0.10),
            new state(states.PressBeacon, team.Red       ),
            new state(states.StrafeRight,   0.25,      1.00), //Away from wall
            new state(states.TurnLeftEnc,   42,       1.00),
            new state(states.Move,          225,      - 1.00),
        };
    public final state[] RedShootBeaconParkWoodArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,       1.00),
            new state(states.TurnLeftEnc,   60,       0.35),
            new state(states.Move,          230,    1.00),
            new state(states.TurnRight,     25,       0.15),
            new state(states.StrafeToWall,  11,         0.10),
            new state(states.TurnRight,     -tolerance,       0.05),
            new state(states.TurnLeft,      tolerance,        0.05),
            new state(states.Move,          50,       1.00),
            new state(states.LineSearch,    2,         0.10),
            new state(states.StrafeToWall,  9,         0.10),
            new state(states.PressBeacon, team.Red                              ),
            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall
            new state(states.Move,          125,     - 0.50),
            new state(states.TurnRight,    -tolerance,         0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,         0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.StrafeToWall,  11,         0.13),
            new state(states.LineSearch,    2,         0.10),
            new state(states.StrafeToWall,  8,         0.10),
            new state(states.PressBeacon, team.Red       ),
            new state(states.StrafeRight,   0.25,      1.00), //Away from wall
            new state(states.TurnRightEnc,  90,       1.00),
            new state(states.ShootAtPower,  0,       1.00),
            new state(states.Move,          60,      - 1.00),
            new state(states.EnableShot,    1500,     1.00),
            new state(states.ShootAtPower,  0,       0.00),
            new state(states.StrafeLeft,    750,       1.00),
            new state(states.TurnRightEnc,  45,     1.00),
            new state(states.Move,  80,     1.00),
    };
    public final state[] RedShootBeaconParkRampArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,       1.00),
            new state(states.TurnLeftEnc,   60,       0.35),
            new state(states.Move,          230,    1.00),
            new state(states.TurnRight,     25,       0.15),
            new state(states.StrafeToWall,  11,         0.10),
            new state(states.TurnRight,     -tolerance,       0.05),
            new state(states.TurnLeft,      tolerance,        0.05),
            new state(states.Move,          90,       1.00),
            new state(states.LineSearch,    2,         0.10),
            new state(states.StrafeToWall,  9,         0.10),
            new state(states.PressBeacon, team.Red       ),
            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall
            new state(states.Move,          125,     - 0.50),
            new state(states.TurnRight,    -tolerance,         0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,         0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.StrafeToWall,  11,         0.13),
            new state(states.LineSearch,    2,         0.10),
            new state(states.StrafeToWall,  8,         0.10),
            new state(states.PressBeacon, team.Red       ),
            new state(states.StrafeRight,   0.10,      1.00), //Away from wall
            new state(states.ShootAtPower,  0,       1.00),
            new state(states.TurnRight,     90,      0.50),
            new state(states.Move,          35,      - 1.00),
            new state(states.EnableShot,    1500,     1.00),
            new state(states.ShootAtPower,  0,       0.00),
            new state(states.StrafeRight,  .075,       1.00),
            new state(states.TurnLeftEnc,  45,     1.00),
            new state(states.Move,          80,   - 1.00),
    };

    public final state[] RedBeaconParkRampArray = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,       1.00),
            new state(states.TurnLeftEnc,   60,       0.25),
            new state(states.Move,          230,    1.00),
            new state(states.TurnRight,     25,       0.15),
            new state(states.StrafeToWall,  11,         0.10),
            new state(states.TurnRight,     -tolerance,       0.05),
            new state(states.TurnLeft,      tolerance,        0.05),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.StrafeToWall,  9,         0.10),
            new state(states.PressBeacon, team.Red       ),
            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall
            new state(states.Move,          125,       0.50),
            new state(states.TurnRight,    -tolerance,         0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,         0.05),
            new state(states.LineSearch,    2,         0.10),
            new state(states.StrafeToWall,  11,         0.13),
            new state(states.LineSearch,    2,       - 0.10),
            new state(states.StrafeToWall,  8,         0.10),
            new state(states.PressBeacon, team.Red       ),
            new state(states.StrafeRight,   0.25,      1.00), //Away from wall
            new state(states.TurnLeftEnc,   42,       1.00),
            new state(states.Move,          225,      - 1.00),
    };

    public final state[] GenericShootFromFarAngleArray = new state[] {
            new state(states.Move,     180,      .75),
            new state(states.Shoot,      0,        0),
            new state(states.Move,      60,       .5),
            new state(states.Move,      60,     - .5),
            new state(states.Move,      60,      1.0),
    };
    public final state[] GenericShootFromNextToRampArray = new state[] {
            new state(states.Move,     180,      .75),
            new state(states.Shoot,      0,        0),
            new state(states.Move,      60,       .5),
            new state(states.Move,      60,     - .5),
            new state(states.Move,      60,      1.0),
    };

    public final state[] DoNothingArray = new state[] {
            new state(states.Finished,0,0)
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
        stopDriveMotors();
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

        int timer = 0;
        int cycler = 0;

        /*
              __  __
             |  \/  |
             | \  / | ___ _ __  _   _
             | |\/| |/ _ \ '_ \| | | |
             | |  | |  __/ | | | |_| |
             |_|  |_|\___|_| |_|\__,_|
        */


        //Based on the value of autoRouteChosen, we set the stateOrder array to the correct value

        String autoRouteChosen = "RedBeaconParkRamp";
        int pos = 0;
        String [] routes = {"RedBeaconParkRamp",
                "RedBeaconParkWood",
                "RedShootBeaconParkRamp",
                "RedShootBeaconParkWood",
                "BlueBeaconParkRamp",
                "BlueBeaconParkWood",
                "BlueShootBeaconParkRamp",
                "BlueShootBeaconParkWood",
                "GenericShootFromFarAngle",
                "GenericShootFromNextToRamp"};
        boolean prevX=false, prevB=false;
        while(!gamepad1.a){
            if(gamepad1.x && !prevX){
                pos--;
            } else if(gamepad1.b && !prevB){
                pos++;
            }
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            if(pos<0){
                pos = routes.length - 1;
            }
            if(pos==routes.length){
                pos = 0;
            }
            autoRouteChosen = routes[pos];
            telemetry.addData("Route", autoRouteChosen);
            telemetry.update();
        }

        stateOrder = DoNothingArray;
        switch (autoRouteChosen){
            case "RedBeaconParkRamp":
                stateOrder = RedBeaconParkRampArray;
                break;
            case "RedBeaconParkWood":
                stateOrder = RedBeaconParkWoodArray;
                break;
            case "RedShootBeaconParkRamp":
                stateOrder = RedShootBeaconParkRampArray;
                break;
            case "RedShootBeaconParkWood":
                stateOrder = RedShootBeaconParkWoodArray;
                break;
            case "BlueBeaconParkRamp":
                stateOrder = BlueBeaconParkRampArray;
                break;
            case "BlueBeaconParkWood":
                stateOrder = BlueBeaconParkWoodArray;
                break;
            case "BlueShootBeaconParkRamp":
                stateOrder = BlueBeaconShootParkRampArray;
                break;
            case "BlueShootBeaconParkWood":
                stateOrder = BlueBeaconShootParkWoodArray;
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
                        }
                        positionInArrayAsWeAdd++;
                    }
                    inputArray[positionInArrayAsWeAdd + 1] = new state(states.Finished, 0, 0); //automatically ends the autonomoous after all commands have been tried
                    stateOrder = inputArray;
                } catch (Exception e){
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

        timer = 0;
        while(gyroSensor.isCalibrating()) {
            timer++;
            if(timer % 15 == 0){
                cycler++;
            }
            if (cycler == 0){
                telemetry.addData("Gyro", " is still Calibrating");
            } else if (cycler == 1) {
                telemetry.addData("Gyro", " is still Calibrating.");
            } else if (cycler == 2){
                telemetry.addData("Gyro", " is still Calibrating..");
            } else {
                cycler = 0;
                telemetry.addData("Gyro", " is still Calibrating...");
            }
            telemetry.update();
        } //This silly looking code above animates a "..." sequence in telemetry, if the gyroscope is still calibrating
        telemetry.addData("Route", Objects.equals(autoRouteChosen, "ReadInput") ? "Custom" : autoRouteChosen);
        if(range.rawUltrasonic() > 0 && colorSensorOnSide.alpha() == 0 && colorSensorLeftBottom.alpha() > 0 && timer < 10000){
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

        while((CurrentState.getState() != states.Finished) && (opModeIsActive())) {
            switch (CurrentState.getState()) {
                // We will be using this switch as our main control loop.
                // The nice thing about this architechture is we can do manual overrides of states,
                // and that we can easily change what state we are running
                case Move:
                    // We want to reset the encoders at the start of each state that uses them.
                    // UpdateState always sets initialized to false, so we will always reset the encoders.
                    // reset encoders also automagically makes initialized true, after 50ms have already reset the encoders.
                    if(!initialized){
                        initEncoders();
                    }
                    time++;
                    if(time < 50){
                        break;
                    }

                    ticks = CurrentState.getSensorValue() / cmPerTick;
                    //Determines how many encoder ticks are required to travel the expected distance.
                    // This calculation is based on the gear ratios and encoder ticks per rotation as defined above

                    setDrivePower(CurrentState.getPowerValue());

                    RBPos = Math.abs(rightBackWheel.getCurrentPosition());
                    RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
                    LBPos = Math.abs(leftBackWheel.getCurrentPosition());
                    LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
                    // Use absolute values of the encoder readings to account for negative powers.

                    Average = (RBPos + RFPos + LBPos + LFPos)/4;
                    currentDistance = Average * cmPerTick; //Using the average and cmPerTick, we can determine the distance the robot has driven.

                    telemetry.addData("Target Distance: ", CurrentState.getSensorValue());
                    telemetry.addData("Current Distance: ", currentDistance);
                    telemetry.update();

                    if(Average >= ticks){
                        setDrivePower(0);
                        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        CurrentState = updateState();
                    }
                    break;
                case TurnLeft:
                    degrees = CurrentState.getSensorValue();
                    power = CurrentState.getPowerValue();
                    if(!initEncoders()){
                        break;
                    }
                    //Same conept as above
                    if(gyroSensor.getIntegratedZValue() >= degrees){
                        setDrivePower(0);
                        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        CurrentState = updateState();
                        break;
                    }

                    rightBackWheel.setPower(power);
                    rightFrontWheel.setPower(power);
                    leftBackWheel.setPower(-power);
                    leftFrontWheel.setPower(-power);
                    //Have the robot spin.

                    break;
                case TurnRight:
                    //Everything here works just like it does in TurnLeft, just spinning in the opposite direction
                    degrees = CurrentState.getSensorValue();
                    power = CurrentState.getPowerValue();
                    if(!initEncoders()){
                        break;
                    }
                    //Same conept as above
                    if(gyroSensor.getIntegratedZValue() <= degrees){
                        setDrivePower(0);
                        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        CurrentState = updateState();
                        break;
                    }

                    rightBackWheel.setPower(-power);
                    rightFrontWheel.setPower(-power);
                    leftBackWheel.setPower(power);
                    leftFrontWheel.setPower(power);
                    //Have the robot spin.

                    break;
                case TurnLeftEnc:
                    degrees = CurrentState.getSensorValue();
                    power = CurrentState.getPowerValue();
                    if(!initEncoders()){
                        break;
                    }
                    //Same conept as above

                    changeFactor = 90;
                    modified = changeFactor + degrees;

                    circleFrac = modified/360;
                    cm = width * circleFrac * Math.PI * 2;
                    movement = cm / cmPerTick;
                    ticks = movement/2;
                    //Use the width of the wheel to determine a value for ticks.

                    rightBackWheel.setPower(power);
                    rightFrontWheel.setPower(power);
                    leftBackWheel.setPower(-power);
                    leftFrontWheel.setPower(-power);
                    //Have the robot spin.

                    RBPos = Math.abs(rightBackWheel.getCurrentPosition());
                    RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
                    LBPos = Math.abs(leftBackWheel.getCurrentPosition());
                    LFPos = Math.abs(leftFrontWheel.getCurrentPosition());

                    Average = (RBPos + RFPos + LBPos + LFPos)/4;

                    int currentHeading = getHeading();
                    telemetry.addData("Target Heading: ", degrees);
                    telemetry.addData("Current Heading: ", currentHeading);
                    telemetry.update();

                    if(Average >= ticks){
                        setDrivePower(0);
                        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        CurrentState = updateState();
                    }
                    break;
                case TurnRightEnc:
                    //Everything here works just like it does in TurnLeft, just spinning in the opposite direction
                    degrees = CurrentState.getSensorValue();
                    power = CurrentState.getPowerValue();
                    if(!initEncoders()){
                        break;
                    }

                    changeFactor = 90;
                    modified = changeFactor + degrees;

                    circleFrac = modified/360;
                    cm = width * circleFrac * Math.PI * 2;
                    movement = cm / cmPerTick;
                    ticks = movement/2;

                    leftBackWheel.setPower(power);
                    leftFrontWheel.setPower(power);
                    rightBackWheel.setPower(-power);
                    rightFrontWheel.setPower(-power);

                    RBPos = Math.abs(rightBackWheel.getCurrentPosition());
                    RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
                    LBPos = Math.abs(leftBackWheel.getCurrentPosition());
                    LFPos = Math.abs(leftFrontWheel.getCurrentPosition());

                    Average = (RBPos + RFPos + LBPos + LFPos)/4;

                    currentHeading = getHeading();
                    telemetry.addData("Target Heading: ", degrees);
                    telemetry.addData("Current Heading: ", currentHeading);
                    telemetry.update();

                    if(Average >= ticks){
                        setDrivePower(0);
                        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        CurrentState = updateState();
                    }
                    break;

                case Shoot:
                    if(!initEncoders()){
                        break;
                    }
                    time++;
                    //The time variable will increment exactly once a millisecond, so our elseifs will function like sleep() methods.
                    if(time < 700){
                        shoot1.setPower(1);
                        shoot2.setPower(1);
                        ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
                        ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
                    } else if(time < (700 + 400)){
                        infeed.setPower(1);
                    } else if(time < (700+400+1000)) {
                        infeed.setPower(0);
                    } else if(time < (700+400+1000+300)){
                        infeed.setPower(1);
                    } else {
                        //once we have passsed 2400 milliseconds, we are done shooting.
                        CurrentState = updateState();
                        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
                        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
                        infeed.setPower(0);
                        shoot1.setPower(0);
                        shoot2.setPower(0);
                    }
                    break;
                case StrafeLeft:
                    Seconds = CurrentState.getSensorValue();
                    Speed = CurrentState.getPowerValue();
                    if(!initEncoders()){
                        break;
                    }
                    time++;

                    Time = Seconds*1000;
                    setStrafePower("turnLeft", Speed);
                    if(time > Time){
                        setDrivePower(0);
                        CurrentState = updateState();
                    }
                    break;
                case StrafeRight:
                    Seconds = CurrentState.getSensorValue();
                    Speed = CurrentState.getPowerValue();
                    if(!initEncoders()){
                        break;
                    }
                    time++;

                    Time = Seconds*1000;
                    setStrafePower("turnRight", Speed);
                    if(time > Time){
                        setDrivePower(0);
                        CurrentState = updateState();
                    }
                    break;
                case StrafeToWall:
                    Speed = CurrentState.getPowerValue();
                    double DistanceReading = CurrentState.getSensorValue();
                    if(!initEncoders()){
                        break;
                    }

                    setStrafePower("turnLeft", Speed);

                    telemetry.addData("Target Distance: ", DistanceReading);
                    telemetry.addData("Current Distance", range.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    if(range.getDistance(DistanceUnit.CM) <=  DistanceReading){
                        setDrivePower(0);
                        CurrentState = updateState();
                    }
                    break;
                case LineSearch:
                    if(!initEncoders()){
                        break;
                    }
                    time++;
                    if(time < 50){
                        break;
                    }
                    Speed = CurrentState.getPowerValue();
                    double expectedReading = CurrentState.getSensorValue();
                    setDrivePower(Speed);
                    int red = colorSensorLeftBottom.red();
                    int blue = colorSensorLeftBottom.blue();
                    int alpha = colorSensorLeftBottom.alpha();

                    if(red >= expectedReading && blue >= expectedReading && alpha >= expectedReading) {
                        //If the readings are above what we want them to be, we have found the line
                        setDrivePower(0);
                        CurrentState= updateState();
                    }
                    break;
                case PressBeacon:
                    time++;
                    redReading = colorSensorOnSide.red();
                    blueReading = colorSensorOnSide.blue();
                    if(time < 100){
                        break;
                    }

                    if((colorReading == team.NotSensed)){
                        if (blueReading > redReading)
                        {
                            colorReading = team.Blue;
                        } else {
                            colorReading = team.Red;
                        }
                    } //We only want to do this once, because the lights flash once we trigger the beacon.

                    if(colorReading == CurrentState.getAlliance())
                    {
                        dim.setLED(0, true); //Red
                        dim.setLED(1, false); //Blue
                        if(!movedServo) {
                            leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
                            movedServo = true;
                        }
                        if(time < 1250)
                            break;
                        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
                        CurrentState = updateState();
                        //The left servo is below the voltage_sensor
                    } else {
                        dim.setLED(0, false); //Red
                        dim.setLED(1, true); //Blue
                        if(!movedServo) {
                            rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
                            movedServo = true;
                        }
                        if(time < 1250)
                            break;
                        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
                        CurrentState = updateState();
                    }
                    break;
                case ShootAtPower:
                    shoot1.setPower(CurrentState.getPowerValue());
                    shoot2.setPower(CurrentState.getPowerValue());
                    CurrentState = updateState();
                    break;
                case EnableShot:
                    time++;
                    ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
                    ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
                    infeed.setPower(CurrentState.getPowerValue());
                    if(time < CurrentState.getSensorValue()){
                        break;
                    }
                    ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
                    ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
                    infeed.setPower(0);
                    CurrentState = updateState();
                    break;
                case WaitForTime:
                    time++;
                    if(time < CurrentState.getSensorValue()){
                        break;
                    }
                    CurrentState = updateState();
                    break;
                case Finished:
                    stopDriveMotors();
                    telemetry.clear();
                    telemetry.addLine("All Done!");
                    telemetry.update();
                    super.stop();
                    break;
                default:
                    telemetry.addLine("Uh oh! Couldn't find a case statement for a state " + CurrentState.getState());
                    telemetry.update();
                    super.stop();
            } //Code Below this point will run on every cycle
            telemetry.clear();
            telemetry.addData("State", CurrentState.getState());

            telemetry.addData("turnLeft Bottom", colorSensorLeftBottom.alpha());
            telemetry.addData("Side Color", colorReading);
            telemetry.addData("G", gyroSensor.getHeading());
            telemetry.update();
//            while(1000*1000 > SystemClock.elapsedRealtimeNanos() - lastTime){
//            } //guarantees a loop speed of 1000 milliseconds.
            lastTime = SystemClock.elapsedRealtimeNanos();
            idle();
        }
        dim.setLED(0, false);
        dim.setLED(1, false);
    }

    public static boolean resetEncoder(DcMotor m){
        m.setMode(DcMotor.RunMode.RESET_ENCODERS);
        if(m.getCurrentPosition()!=0){
            return false;
        }//wait
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return true;
    }

    public void stopDriveMotors(){
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

    public boolean initEncoders()  throws InterruptedException {
        if(initialized)
            return true;
        if(resetEncoder(leftBackWheel) && resetEncoder(leftFrontWheel) && resetEncoder(rightBackWheel) && resetEncoder(rightFrontWheel)){
            initialized = true;
            time = 0;
            return true;
        }
        return false;
    }

    public void setDrivePower(double power) {
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
    }

    public int getHeading() {

        int currentHeading = gyroSensor.getHeading();

        int output = currentHeading;

        if(currentHeading > 180 && currentHeading < 360) {
            output = currentHeading - 360;
        }

        return output;
    }

    public void setStrafePower(String Direction, double Speed) {
        if(Objects.equals(Direction, "turnLeft"))
        {
            rightFrontWheel.setPower(Speed);
            rightBackWheel.setPower(-Speed);
            leftFrontWheel.setPower(-Speed);
            leftBackWheel.setPower(Speed);
        }
        if(Objects.equals(Direction, "turnRight"))
        {
            rightFrontWheel.setPower(-Speed);
            rightBackWheel.setPower(Speed);
            leftFrontWheel.setPower(Speed);
            leftBackWheel.setPower(-Speed);
        }
    }


}
