package org.firstinspires.ftc.teamcode.reference;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

import static java.lang.Thread.sleep;

/**
 * Created by Ethan Schaffer on 11/17/2016.
 */

@Autonomous(name="State Machine", group="Autonomous")
@Disabled
public class stateMachineBigLoop extends LinearOpMode {
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
        ForwardsToShoot, Shooting, BackingUp, Angling, ForwardsToWall, AlignAgainstWall, GoingToBeacon1,  GoingToBeacon2, PressingBeacon, CapBall, Finished
    }
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
    public enum goingToBeaconState{
        Going, Correction, Angle;
    }
    // Shoot
    public static final double POWER1 = -.5, DISTANCE1 = 1900;           //Distance Forwards before shot
    public static final long SHOTTIMEINMILLISECONDS = 1500;              //How long we shoot the ball
//    public static final long SHOT_TIME_IN_MILLISECONDS = 4500;              //How long we shoot the ball

    //Line up with wall
    public static final double POWER2 = .2,  DISTANCE2 = 500;             //Backup Distance
    public static final double POWER3 = .15,  GYRO3 = 40; //Turn Reading
    public static final double POWER4 = -.5, DISTANCE4 = 3200;            //Forwards Distance
    public static final double POWER5 = -.15, GYRO5 = 0; //Turn Reading

    //Allign with wall by running into it
    public static final double POWER6 = .45, DISTANCE6 = 500;    //Strafe to between the white lines.
    public static final double CM_FROM_WALL_VALUE = 4;

    //Getting Beacons
    public static final double POWER7 = .15, COLOR_READING_FOR_LINE = 3; //Backwards to beacon #1
    public static final double POWER_HANDLE_COLOR = .15;
    public static final double POWER8 = -.2, DISTANCE8 = 250; //Forwards to pressBeacon #2

    //Cap Ball
    public static  final double POWER9 = .2, GYRO9 = 45; //ReOrient for Hitting Cap Ball
    public static final double POWER10 = .5, DISTANCE10 = 2750; //Distance to Ball
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
    goingToBeaconState gState = goingToBeaconState.Angle;
    double lPower;
    double rPower;
    long lastTime;
    long time;
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
        CurrentState = state.ForwardsToShoot;

        waitForStart();
        resetEncoder(leftFrontWheel);
        while((CurrentState != state.Finished) && (opModeIsActive())) {
            switch (CurrentState) {
                case ForwardsToShoot:
                    arcade(POWER1, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                    if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE1) {
                        stopMotors();
                        PreviousState = state.ForwardsToShoot;
                        CurrentState = state.Shooting;
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
                            if(time > SHOTTIMEINMILLISECONDS){
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
                                CurrentState = state.BackingUp;
                                PreviousState = state.Shooting;
                            }
                            break;
                        default:
                            break;
                    }
                    break;

                case BackingUp:
                    PreviousState = state.BackingUp;
                    waitForMS(TIME_WAIT_SMALL);
                    arcade(POWER2, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                    if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE2) {
                        stopMotors();
                        CurrentState = state.Angling;
                        arcade(0, 0, POWER3, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                    }
                    break;

                case Angling:
                    //TURN
                    if (gyroSensor.getIntegratedZValue() >= GYRO3) {
                        stopMotors();
                        if(resetEncoder(leftFrontWheel)){
                            CurrentState = state.ForwardsToWall;
                            PreviousState = state.Angling;
                            arcade(POWER4, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                        }
                    }
                    break;
                case ForwardsToWall:
                    //Forwards
                    if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE4) {
                        stopMotors();
                        CurrentState = state.AlignAgainstWall;
                        PreviousState = state.ForwardsToWall;
                        aState = aligningStates.LineToAngle;
                    }
                    break;

                case AlignAgainstWall:
                    //TURN BACK
                    switch (aState){
                        case LineToAngle:
                            arcade(0, 0, POWER5, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if(gyroSensor.getIntegratedZValue() <= GYRO5){
                                aState = aligningStates.RangeToWall;
                                stopMotors();
                            }
                            break;
                        case RangeToWall:
                            arcade(0, POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if (range.getDistance(DistanceUnit.CM) <= CM_FROM_WALL_VALUE) {
                                stopMotors();
                                if(resetEncoder(leftFrontWheel)){
                                    aState = aligningStates.BangBangLineup;
                                    arcade(0, POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                                }
                            }
                            break;
                        case BangBangLineup:
                            if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE6) {
                                stopMotors();
                                if(resetEncoder(leftFrontWheel)){
                                    aState = aligningStates.Away;
                                    arcade(0, -POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                                }
                            }
                            break;
                        case Away:
                            if (Math.abs(leftFrontWheel.getCurrentPosition()) >= (DISTANCE6 * .75)) {
                                stopMotors();
                                PreviousState = state.AlignAgainstWall;
                                time = 0;
                                CurrentState = state.GoingToBeacon1;

//                                CurrentState = states.Finished;
                            }
                            break;
                        default:
                            super.stop();
                    }
                    break;
                case GoingToBeacon1:
                    //backwards to back color voltage_sensor
                    switch(gState){
                        case Going:
                            double pValue = .015;
                            if(time <= 0){
                                lPower = POWER7 - ((gyroSensor.getIntegratedZValue() * pValue));
                                rPower = POWER7 + ((gyroSensor.getIntegratedZValue() * pValue));
                                leftBackWheel.setPower(lPower);
                                leftFrontWheel.setPower(lPower);
                                rightBackWheel.setPower(rPower);
                                rightFrontWheel.setPower(rPower);
                            }
                            if ( ( (colorSensorLeftBottom.alpha() > COLOR_READING_FOR_LINE) && (colorSensorLeftBottom.alpha() != 255) )  || ((colorSensorRightBottom.alpha() > COLOR_READING_FOR_LINE) )&& colorSensorRightBottom.alpha() != 255 ) {
                                time++;
                                stopMotors();
                                if(time > 500)
                                    gState = goingToBeaconState.Correction;
                            }
                            break;
                        case Correction:
                            pValue = .01;
                            lPower = POWER7 - ((gyroSensor.getIntegratedZValue()* pValue));
                            rPower = POWER7 + ((gyroSensor.getIntegratedZValue()* pValue));
                            leftBackWheel.setPower(lPower);
                            leftFrontWheel.setPower(lPower);
                            rightBackWheel.setPower(rPower);
                            rightFrontWheel.setPower(rPower);
                            if ( ( (colorSensorLeftBottom.alpha() > COLOR_READING_FOR_LINE) && (colorSensorLeftBottom.alpha() != 255) )  || ((colorSensorRightBottom.alpha() > COLOR_READING_FOR_LINE) )&& colorSensorRightBottom.alpha() != 255 ) {
                                stopMotors();
                                PreviousState = state.GoingToBeacon1;
                                CurrentState = state.PressingBeacon;
                            }
                            break;
                        case Angle:
                            arcade(0, 0, POWER5, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if(gyroSensor.getIntegratedZValue() <= GYRO5){
                                gState = goingToBeaconState.Going;
                                stopMotors();
                            }
                            break;
                    }
                    break;
                case GoingToBeacon2:
                    PreviousState = state.GoingToBeacon1;
                    //backwards to back color voltage_sensor
                    double pValue = 25;
                    lPower = POWER8 + ((gyroSensor.getIntegratedZValue() / pValue));
                    rPower = POWER8 - ((gyroSensor.getIntegratedZValue() / pValue));
                    lPower = POWER8;
                    rPower = POWER8;
                    leftBackWheel.setPower(lPower);
                    leftFrontWheel.setPower(lPower);
                    rightBackWheel.setPower(rPower);
                    rightFrontWheel.setPower(rPower);

                    if ((colorSensorLeftBottom.alpha() > COLOR_READING_FOR_LINE) || (colorSensorRightBottom.alpha() > COLOR_READING_FOR_LINE)) {
                        stopMotors();
                        CurrentState = state.PressingBeacon;
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
                                pState = pressingState.Finished;
                                time = 0;
                            }
                            break;
                        case Finished:
                            time++;
                            if (PreviousState == state.GoingToBeacon1) {
                                pValue = 25;
                                lPower = POWER8 + ((gyroSensor.getIntegratedZValue() / pValue));
                                rPower = POWER8 - ((gyroSensor.getIntegratedZValue() / pValue));
                                lPower = POWER8;
                                rPower = POWER8;
                                leftBackWheel.setPower(lPower);
                                leftFrontWheel.setPower(lPower);
                                rightBackWheel.setPower(rPower);
                                rightFrontWheel.setPower(rPower);
                                if(time > 250){
                                    CurrentState = state.GoingToBeacon2;
                                }
                            } else {
                                CurrentState = state.CapBall;
                            }
                            PreviousState = state.PressingBeacon;
                            CurrentState = state.Finished;
                            break;
                    }
                    break;
                case CapBall:
                    switch (cState){
                        case Rotate:
                            arcade(0, 0, POWER9, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if (gyroSensor.getIntegratedZValue() >= GYRO9) {
                                stopMotors();
                                if(resetEncoder(leftFrontWheel)){
                                    cState = capStates.Backwards;
                                }
                            }
                            break;
                        case Backwards:
                            resetEncoder(leftFrontWheel);
                            arcade(POWER10, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                            if (Math.abs(leftFrontWheel.getCurrentPosition()) >= DISTANCE10) {
                                PreviousState = state.CapBall;
                                CurrentState = state.Finished;
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
                    if(CurrentState == state.CapBall){
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
            if(CurrentState == state.CapBall){
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
            while(1000*1000 > SystemClock.elapsedRealtimeNanos() - lastTime){

            } //guarantees a loop speed of 1000 milliseconds.
            lastTime = SystemClock.elapsedRealtimeNanos();
        }
        dim.setLED(0, false);
        dim.setLED(1, false);
    }

    public void waitForMS(long ms) throws InterruptedException {
        Thread.sleep(ms);
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
