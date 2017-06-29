package org.firstinspires.ftc.teamcode.reference;

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

import java.util.Arrays;

/**
 * Created by Ethan Schaffer on 11/17/2016.
 */

@Autonomous(name="State Machine (Old)", group="Autonomous")
@Disabled
public class stateMachineSmarter extends LinearOpMode {
    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String BALLBLOCKLEFTNAME = "bl", BALLBLOCKRIGHTNAME = "br"; //MO Ports 3+4
    public static final double BALLBLOCKLEFTOPEN = 1, BALLBLOCKLEFTCLOSED = 0;
    public static final double BALLBLOCKRIGHTOPEN = 0, BALLBLOCKRIGHTCLOSED = 1;
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String GYRONAME = "g"; //Port 4
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 1
    public static final String COLORLEFTBOTTOMNAME = "cb";//Port 2
    public static final String COLORRIGHTBOTTOMNAME = "cb2"; //Port 4

    public static final double LEFT_SERVO_OFF_VALUE = .3;
    public static final double LEFT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_OFF_VALUE = .3;


    //TODO: make a pretty 'map'

    public enum state{
        ForwardsToShoot, Shooting, BackingUp, Angling, ForwardsToWall, AlignAgainstWall, GoingToBeacon1,  GoingToBeacon2, PressingBeacon, CapBallFromFirstBeacon, CapBallFromSecondBeacon, Wait5Seconds, Finished
    }
    state[] stateOrder = new state[]{state.ForwardsToShoot, state.Shooting, state.BackingUp, state.Angling, state.ForwardsToWall, state.AlignAgainstWall,
            state.GoingToBeacon1, state.PressingBeacon, state.GoingToBeacon2, state.PressingBeacon, state.CapBallFromSecondBeacon, state.Finished};

    public enum aligningStates{
        LineToAngle, RangeToWall, BangBangLineup, Away
    }
    public enum capStates{
        Rotate, Backwards;
    }
    public enum shootState{
        Init, InfeedOn, OpenServo, Done;
    }
    public enum pressingState{
        Init, Pressing, DonePressing, Finished;
    }
    public enum waitState{
        Init, Done;
    }
    public enum goingToBeaconState{
        Going, StrafeIn, Correction, Angle;
    }
    // Shoot
    public static final double POWER_TO_SHOT = -.5, DISTANCE_TO_SHOT = 1900;           //Distance Forwards before shot
    public static final long SHOT_TIME_IN_MILLISECONDS = 1500;              //How long we shoot the ball
//    public static final long SHOT_TIME_IN_MILLISECONDS = 4500;              //How long we shoot the ball

    //Line up with wall
    public static final double POWER_BACK_FROM_SHOT = .2,  DISTANCE_BACK_FROM_SHOT = 500;             //Backup Distance
    public static final double POWER_TURNING = .15,  GYRO_TARGET_TURN_TOWARDS_WALL = 40; //Turn Reading
    public static final double POWER_TOWARDS_WALL = -.5, DISTANCE_TOWARDS_WALL = 3200;            //Forwards Distance
    public static final double POWER_TURNING_BACK = -POWER_TURNING, GYRO_TARGET_TURNING_BACK = 0; //Turn Reading

    //Allign with wall by running into it
    public static final double POWER_HIT_WALL = .5, DISTANCE_TO_HIT_WALL = 500;    //Strafe to between the white lines.
    public static final double CM_FROM_WALL_VALUE_FIRST_STRAFE = 4;

    //Getting Beacons
    public static final double POWER_TO_GET_TO_BEACON1 = .15;
    public static final double POWER_TO_GET_TO_BEACON2 = -POWER_TO_GET_TO_BEACON1, COLOR_READING_FOR_LINE = 3; //Backwards to beacon #1
    public static final double POWER_BEFORE_DISTANCE_REACHED = -.4, DISTANCE_BEFORE_ENABLE_COLOR_SENSOR = 500, POWER_AFTER_DISTANCE_REACHED = -.2; //Forwards to pressBeacon #2

    //Cap Ball
    public static final double POWER_CAP_REORIENT_FROM_BEACON2 = .2, GYRO_CAP_REORIENT_FROM_BEACON2 = 45; //ReOrient for Hitting Cap Ball
    public static final double POWER_CAP_HIT_FROM_BEACON2 = .5, DISTANCE_CAP_HIT_FROM_BEACON2 = 2750; //Distance to Ball

    public static final double POWER_CAP_REORIENT_FROM_BEACON1 = .2, GYRO_CAP_REORIENT_FROM_BEACON1 = 90; //ReOrient for Hitting Cap Ball
    public static final double POWER_CAP_HIT_FROM_BEACON1 = .5, DISTANCE_CAP_HIT_FROM_BEACON1 = 1750; //Distance to Ball

    public static final long TIME_WAIT_SMALL = 50, TIME_WAIT_MEDIUM = 500, TIME_WAIT_LARGE = 1000;

    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlockRight, ballBlockLeft;
    ColorSensor colorSensorLeftBottom, colorSensorRightBottom, colorSensorOnSide;
    ModernRoboticsI2cRangeSensor range;
    ModernRoboticsI2cGyro gyroSensor;
    DeviceInterfaceModule dim;
    state CurrentState, PreviousState;
    aligningStates aState = aligningStates.LineToAngle;
    capStates cState = capStates.Rotate;
    shootState sState = shootState.Init;
    pressingState pState = pressingState.Init;
    waitState wState = waitState.Init;
    goingToBeaconState gState = goingToBeaconState.Angle;
    double lPower;
    double rPower;
    long lastTime;
    long time;

    public int stateOrderPos = -1;
    public state getNextState(){
        stateOrderPos++;
        return stateOrder[stateOrderPos];
    }
    public state getCurrentState(){
        return stateOrder[stateOrderPos];
    }
    public state getFutureState(){
        try {
            return stateOrder[stateOrderPos+1];
        } catch (IndexOutOfBoundsException e) { //Avoid index out of bounds
            return state.Finished;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontWheel = hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(RIGHT2NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

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
        dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRONAME);

        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Gyro", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Calibrated!");

        telemetry.addData("Raw Ultrasonic", range.rawUltrasonic());
        telemetry.addData("Color Side Red", colorSensorOnSide.red());
        telemetry.addData("Color turnLeft Alpha", colorSensorLeftBottom.alpha());
        telemetry.addData("Color turnRight Alpha", colorSensorRightBottom.alpha());
        telemetry.update();
        CurrentState = getNextState();

        waitForStart();
        double totalTime = 0;
        resetEncoder(leftFrontWheel);
        while((CurrentState != state.Finished) && (opModeIsActive()) && (totalTime < (30*1000))) {
            totalTime++;
            switch (CurrentState) {
                case Wait5Seconds:
                    switch(wState){
                        case Init:
                            time = 0;
                            wState = waitState.Done;
                            break;
                        case Done:
                            time++;
                            if(time >= (5*1000)){
                                time = 0;
                                PreviousState = getCurrentState();
                                CurrentState = getNextState();
                            }
                            break;
                    }
                    break;
                case ForwardsToShoot:
                    arcade(POWER_TO_SHOT, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                    if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE_TO_SHOT) {
                        stopMotors();
                        PreviousState = getCurrentState();
                        CurrentState = getNextState();
                    }
                    break;

                case Shooting:
                    switch (sState){
                        case Init:
                            time = 0;
                            infeed.setPower(1);
                            shoot1.setPower(1);
                            shoot2.setPower(1);
                            sState = shootState.InfeedOn;
                        case InfeedOn:
                            time++;
                            if(time > TIME_WAIT_MEDIUM){
                                sState = shootState.OpenServo;
                                ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
                                ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
                                time = 0;
                            }
                            break;
                        case OpenServo:
                            time++;
                            if(time > SHOT_TIME_IN_MILLISECONDS){
                                infeed.setPower(0);
                                shoot1.setPower(0);
                                shoot2.setPower(0);
                                ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
                                ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
                                sState = shootState.Done;
                            }
                            break;
                        case Done:
                            if(resetEncoder(leftFrontWheel)){
                                PreviousState = getCurrentState();
                                CurrentState = getNextState();
                            }
                            break;
                        default:
                            break;
                    }
                    break;

                case BackingUp:
                    waitForMS(TIME_WAIT_SMALL);
                    arcade(POWER_BACK_FROM_SHOT, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                    if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE_BACK_FROM_SHOT) {
                        stopMotors();
                        PreviousState = getCurrentState();
                        CurrentState = getNextState();
                        arcade(0, 0, POWER_TURNING, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                    }
                    break;

                case Angling:
                    //TURN
                    if (gyroSensor.getIntegratedZValue() >= GYRO_TARGET_TURN_TOWARDS_WALL) {
                        stopMotors();
                        if(resetEncoder(leftFrontWheel)){
                            PreviousState = getCurrentState();
                            CurrentState = getNextState();
                            arcade(POWER_TOWARDS_WALL, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                        }
                    }
                    break;
                case ForwardsToWall:
                    //Forwards
                    if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE_TOWARDS_WALL) {
                        stopMotors();
                        PreviousState = getCurrentState();
                        CurrentState = getNextState();
                        aState = aligningStates.LineToAngle;
                    }
                    break;

                case AlignAgainstWall:
                    //TURN BACK
                    switch (aState){
                        case LineToAngle:
                            arcade(0, 0, POWER_TURNING_BACK, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if(gyroSensor.getIntegratedZValue() <= GYRO_TARGET_TURNING_BACK){
                                aState = aligningStates.RangeToWall;
                                stopMotors();
                            }
                            break;
                        case RangeToWall:
                            arcade(0, POWER_HIT_WALL, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if (range.getDistance(DistanceUnit.CM) <= CM_FROM_WALL_VALUE_FIRST_STRAFE) {
                                stopMotors();
                                if(resetEncoder(leftFrontWheel)){
                                    aState = aligningStates.BangBangLineup;
                                    arcade(0, POWER_HIT_WALL, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                                }
                            }
                            break;
                        case BangBangLineup:
                            if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE_TO_HIT_WALL) {
                                stopMotors();
                                if(resetEncoder(leftFrontWheel)){
                                    aState = aligningStates.Away;
                                    arcade(0, -POWER_HIT_WALL, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                                }
                            }
                            break;
                        case Away:
                            if (Math.abs(leftFrontWheel.getCurrentPosition()) >= (DISTANCE_TO_HIT_WALL * .75)) {
                                stopMotors();
                                PreviousState = getCurrentState();
                                CurrentState = getNextState();
                                time = 0;
                            }
                            break;
                        default:
                            super.stop();
                    }
                    break;
                case GoingToBeacon1:
                    //backwards to back color voltage_sensor
                    switch(gState){
                        case Angle:
                            arcade(0, 0, POWER_TURNING_BACK, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if(gyroSensor.getIntegratedZValue() <= GYRO_TARGET_TURNING_BACK){
                                gState = goingToBeaconState.Going;
                                time = 0;
                                stopMotors();
                            }
                            break;
                        case Going:
                            double pValue = .015;
                            if(time <= 0){
                                lPower = POWER_TO_GET_TO_BEACON2 - ((gyroSensor.getIntegratedZValue() * pValue));
                                rPower = POWER_TO_GET_TO_BEACON2 + ((gyroSensor.getIntegratedZValue() * pValue));
                                leftBackWheel.setPower(lPower);
                                leftFrontWheel.setPower(lPower);
                                rightBackWheel.setPower(rPower);
                                rightFrontWheel.setPower(rPower);
                            }
                            if ( ( (colorSensorLeftBottom.alpha() > COLOR_READING_FOR_LINE) && (colorSensorLeftBottom.alpha() != 255) )  || ((colorSensorRightBottom.alpha() > COLOR_READING_FOR_LINE) )&& colorSensorRightBottom.alpha() != 255 ) {
                                time++;
                                stopMotors();
                                if(time > 250){
                                    gState = goingToBeaconState.StrafeIn;
                                    time = 0;
                                }
                            }
                            break;
                        case StrafeIn:
                            arcade(.5, 1, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if(range.getDistance(DistanceUnit.CM) < 7){
                                stopMotors();
                                time++;
                                if(time > 250){
                                    gState = goingToBeaconState.Correction;
                                }
                            }
                            break;
                        case Correction:
                            pValue = .01;
                            lPower = POWER_TO_GET_TO_BEACON2 - ((gyroSensor.getIntegratedZValue()* pValue));
                            rPower = POWER_TO_GET_TO_BEACON2 + ((gyroSensor.getIntegratedZValue()* pValue));
                            leftBackWheel.setPower(lPower);
                            leftFrontWheel.setPower(lPower);
                            rightBackWheel.setPower(rPower);
                            rightFrontWheel.setPower(rPower);
                            if ( ( (colorSensorLeftBottom.alpha() > COLOR_READING_FOR_LINE) && (colorSensorLeftBottom.alpha() != 255) )  || ((colorSensorRightBottom.alpha() > COLOR_READING_FOR_LINE) )&& colorSensorRightBottom.alpha() != 255 ) {
                                stopMotors();
                                PreviousState = getCurrentState();
                                CurrentState = getNextState();
                            }
                            break;
                    }
                    break;
                case GoingToBeacon2:
                    //backwards to back color voltage_sensor
                    switch(gState){
                        case Angle:
                            arcade(0, 0, POWER_TURNING_BACK, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if(gyroSensor.getIntegratedZValue() <= GYRO_TARGET_TURNING_BACK){
                                gState = goingToBeaconState.Going;
                                time = 0;
                                stopMotors();
                            }
                            break;
                        case Going:
                            double pValue = .015;
                            if(time <= 0){
                                lPower = POWER_BEFORE_DISTANCE_REACHED + ((gyroSensor.getIntegratedZValue() * pValue));
                                rPower = POWER_BEFORE_DISTANCE_REACHED - ((gyroSensor.getIntegratedZValue() * pValue));
                                leftBackWheel.setPower(lPower);
                                leftFrontWheel.setPower(lPower);
                                rightBackWheel.setPower(rPower);
                                rightFrontWheel.setPower(rPower);
                            }
                            if ( ( (colorSensorLeftBottom.alpha() > COLOR_READING_FOR_LINE) && (colorSensorLeftBottom.alpha() != 255) )  || ((colorSensorRightBottom.alpha() > COLOR_READING_FOR_LINE) )&& colorSensorRightBottom.alpha() != 255 ) {
                                time++;
                                stopMotors();
                                if(time > 250){
                                    gState = goingToBeaconState.StrafeIn;
                                    time = 0;
                                }
                            }
                            break;
                        case StrafeIn:
                            arcade(-.5, 1, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if(range.getDistance(DistanceUnit.CM) < 7){
                                stopMotors();
                                time++;
                                if(time > 250){
                                    gState = goingToBeaconState.Correction;
                                }
                            }
                            break;
                        case Correction:
                            pValue = .01;
                            lPower = POWER_BEFORE_DISTANCE_REACHED + ((gyroSensor.getIntegratedZValue()* pValue));
                            rPower = POWER_BEFORE_DISTANCE_REACHED - ((gyroSensor.getIntegratedZValue()* pValue));
                            leftBackWheel.setPower(lPower);
                            leftFrontWheel.setPower(lPower);
                            rightBackWheel.setPower(rPower);
                            rightFrontWheel.setPower(rPower);
                            if ( ( (colorSensorLeftBottom.alpha() > COLOR_READING_FOR_LINE) && (colorSensorLeftBottom.alpha() != 255) )  || ((colorSensorRightBottom.alpha() > COLOR_READING_FOR_LINE) )&& colorSensorRightBottom.alpha() != 255 ) {
                                stopMotors();
                                PreviousState = getCurrentState();
                                CurrentState = getNextState();
                            }
                            break;
                    }
                    break;
                case PressingBeacon:
                    //Body
                    dim.setLED(0, true);
                    dim.setLED(1, true);
                    stopMotors();
                    switch (pState){
                        case Init:
                            time = 0;
                            dim.setLED(0, true);
                            dim.setLED(1, true);
                            if (colorSensorOnSide.blue() > colorSensorOnSide.red()) {
                                rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
                                leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
                            } else {
                                rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
                                leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
                            }
                            pState = pressingState.Pressing;
                        case Pressing:
                            time++;
                            if(time > 1500){
                                pState = pressingState.DonePressing;
                                time = 0;
                            }
                            break;
                        case DonePressing:
                            time++;
                            dim.setLED(0, false);
                            dim.setLED(1, false);
                            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
                            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
                            if(time > 500){
                                if(resetEncoder(leftFrontWheel))
                                    pState = pressingState.Finished;
                            }
                            break;
                        case Finished:
                            time++;
                            if ((getFutureState() == state.GoingToBeacon2)) {
                                double pValue = .015;
                                lPower = POWER_TO_GET_TO_BEACON2 + ((gyroSensor.getIntegratedZValue() * pValue));
                                rPower = POWER_TO_GET_TO_BEACON2 - ((gyroSensor.getIntegratedZValue() * pValue));
                                lPower = POWER_TO_GET_TO_BEACON2;
                                rPower = POWER_TO_GET_TO_BEACON2;
                                leftBackWheel.setPower(lPower);
                                leftFrontWheel.setPower(lPower);
                                rightBackWheel.setPower(rPower);
                                rightFrontWheel.setPower(rPower);
                                if (Math.abs(leftFrontWheel.getCurrentPosition()) > DISTANCE_BEFORE_ENABLE_COLOR_SENSOR) {
                                    CurrentState = getNextState();
                                }
                            } else if((getFutureState() == state.GoingToBeacon1) ){
                                double pValue =.015;
                                lPower = POWER_TO_GET_TO_BEACON1 - ((gyroSensor.getIntegratedZValue() * pValue));
                                rPower = POWER_TO_GET_TO_BEACON1 + ((gyroSensor.getIntegratedZValue() * pValue));
                                lPower = POWER_TO_GET_TO_BEACON1;
                                rPower = POWER_TO_GET_TO_BEACON1;
                                leftBackWheel.setPower(lPower);
                                leftFrontWheel.setPower(lPower);
                                rightBackWheel.setPower(rPower);
                                rightFrontWheel.setPower(rPower);
                                if (Math.abs(leftFrontWheel.getCurrentPosition()) > DISTANCE_BEFORE_ENABLE_COLOR_SENSOR) {
                                    CurrentState = getNextState();
                                }
                            } else {
                                if(time > 250){
                                    CurrentState = getNextState();
                                }
                            }
                            PreviousState = getCurrentState();
                            CurrentState = getNextState();
                            break;
                    }
                    break;

                case CapBallFromFirstBeacon:
                    switch (cState){
                        case Rotate:
                            arcade(0, 0, POWER_CAP_REORIENT_FROM_BEACON1, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if (gyroSensor.getIntegratedZValue() >= GYRO_CAP_REORIENT_FROM_BEACON1) {
                                stopMotors();
                                if(resetEncoder(leftFrontWheel)){
                                    cState = capStates.Backwards;
                                }
                            }
                            break;
                        case Backwards:
                            resetEncoder(leftFrontWheel);
                            arcade(POWER_CAP_HIT_FROM_BEACON1, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if (Math.abs(leftFrontWheel.getCurrentPosition()) >= GYRO_CAP_REORIENT_FROM_BEACON1) {
                                PreviousState = getCurrentState();
                                CurrentState = getNextState();
                                stopMotors();
                            }
                            break;
                        default:
                            stopMotors();
                            CurrentState = state.Finished;
                    }
                    break;

                case CapBallFromSecondBeacon:
                    switch (cState){
                        case Rotate:
                            arcade(0, 0, POWER_CAP_REORIENT_FROM_BEACON2, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if (gyroSensor.getIntegratedZValue() >= GYRO_CAP_REORIENT_FROM_BEACON2) {
                                stopMotors();
                                if(resetEncoder(leftFrontWheel)){
                                    cState = capStates.Backwards;
                                }
                            }
                            break;
                        case Backwards:
                            resetEncoder(leftFrontWheel);
                            arcade(POWER_CAP_HIT_FROM_BEACON2, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE_CAP_HIT_FROM_BEACON2) {
                                PreviousState = getCurrentState();
                                CurrentState = getNextState();
                                stopMotors();
                            }
                            break;
                        default:
                            stopMotors();
                            CurrentState = state.Finished;
                    }
                    break;
                case Finished:
                    telemetry.clear();
                    telemetry.addData("State", CurrentState);
                    if(CurrentState == state.AlignAgainstWall){
                        telemetry.addData("Substate", aState);
                    }
                    if(CurrentState == state.CapBallFromSecondBeacon){
                        telemetry.addData("Substate", cState);
                    }
                    if(CurrentState == state.Shooting){
                        telemetry.addData("Substate", sState);
                    }
                    if(CurrentState == state.PressingBeacon){
                        telemetry.addData("Substate", pState);
                    }
                    if(CurrentState == state.GoingToBeacon1){
                        telemetry.addData("Substate", gState);
                    }

                    telemetry.addData("Recent State", PreviousState);
                    telemetry.addData("turnLeft Bottom", colorSensorLeftBottom.alpha());
                    telemetry.addData("turnRight Bottom", colorSensorRightBottom.alpha());
                    telemetry.addData("Side Color", colorSensorOnSide.red() > 2 || colorSensorOnSide.blue() > 2
                            ? colorSensorOnSide.red() > colorSensorOnSide.blue() ? "Blue" : "Red" : "Not Sure");
                    telemetry.addData("G", gyroSensor.getHeading());
                    telemetry.update();

    //                super.stop();
                    break;
                default:
                    super.stop();
            } //Code Below this point will run on every cycle
            telemetry.clear();
            telemetry.addData("State", CurrentState);
            if(CurrentState == state.AlignAgainstWall){
                telemetry.addData("Substate", aState);
            }
            if(CurrentState == state.CapBallFromSecondBeacon){
                telemetry.addData("Substate", cState);
            }
            if(CurrentState == state.Shooting){
                telemetry.addData("Substate", sState);
            }
            if(CurrentState == state.PressingBeacon){
                telemetry.addData("Substate", pState);
            }
            if(CurrentState == state.GoingToBeacon1){
                telemetry.addData("Substate", gState);
            }
            telemetry.addData("Recent State", PreviousState);
            telemetry.addData("Uptime", (totalTime/1000)+" seconds");

            telemetry.addData("turnLeft Bottom", colorSensorLeftBottom.alpha());
            telemetry.addData("turnRight Bottom", colorSensorRightBottom.alpha());
            telemetry.addData("Side Color", colorSensorOnSide.red() > 2 || colorSensorOnSide.blue() > 2
                    ? colorSensorOnSide.red() > colorSensorOnSide.blue() ? "Blue" : "Red" : "Not Sure");
            telemetry.addData("G", gyroSensor.getHeading());
            telemetry.update();
            while(1000*1000 > SystemClock.elapsedRealtimeNanos() - lastTime){

            } //guarantees a loop speed of 1000 milliseconds.
            lastTime = SystemClock.elapsedRealtimeNanos();
        }
        dim.setLED(0, false);
        dim.setLED(1, false);
    }

    public void waitForMS(long ms) throws InterruptedException {
        sleep(0);
//            sleep(ms); //Commented out because sleep statements are evil
    }

    public static boolean resetEncoder(DcMotor m){
        m.setMode(DcMotor.RunMode.RESET_ENCODERS);
        if(m.getCurrentPosition()!=0){
            return false;
        }//wait
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return true;
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
