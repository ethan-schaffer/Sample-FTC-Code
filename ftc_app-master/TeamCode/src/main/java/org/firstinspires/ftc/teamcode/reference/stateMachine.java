package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
public class stateMachine extends OpMode {
    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String BALLBLOCKLEFTNAME = "bl", BALLBLOCKRIGHTNAME = "br"; //MO Ports 3+4
    public static final double BALLBLOCKLEFTOPEN = 0, BALLBLOCKLEFTCLOSED = 1;
    public static final double BALLBLOCKRIGHTOPEN = 1, BALLBLOCKRIGHTCLOSED = 0;
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
    // Shoot
    public static final double POWER1 = -.5, DISTANCE1 = 1900;           //Distance Forwards before shot
    public static final long SHOTTIMEINMILLISECONDS = 500;              //How long we shoot the ball
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
    public static final double POWER7 = .15, COLOR_READING_FOR_LINE = 4; //Backwards to beacon #1
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
    state CurrentState, RecentState;

    @Override
    public void init() {
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
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRONAME);
        dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Gyro", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Calibrated!");

        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();


        CurrentState = state.ForwardsToShoot;
    }

    public void waitForMS(long ms){
        try {
            sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    @Override
    public void loop() {
        switch(CurrentState) {
            case ForwardsToShoot:
                RecentState = state.ForwardsToShoot;
                resetEncoder(leftFrontWheel);


                while (Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE1) {
                    arcade(POWER1, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();
                CurrentState = state.Shooting;
                break;

            case Shooting:
                RecentState = state.Shooting;
                ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
                ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
                waitForMS(TIME_WAIT_SMALL);
                shoot1.setPower(1);
                shoot2.setPower(1);
                waitForMS(TIME_WAIT_MEDIUM);
                infeed.setPower(1);
                waitForMS(SHOTTIMEINMILLISECONDS);
                infeed.setPower(0);
                shoot1.setPower(0);
                shoot2.setPower(0);
                ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
                ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
                CurrentState = state.BackingUp;
                break;

            case BackingUp:
                RecentState = state.BackingUp;
                resetEncoder(leftFrontWheel);
                waitForMS(TIME_WAIT_SMALL);
                while (Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE2) {
                    arcade(POWER2, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();
                CurrentState = state.Angling;
                break;

            case Angling:
                RecentState = state.Angling;
                //TURN
                while (gyroSensor.getIntegratedZValue() < GYRO3) {
                    arcade(0, 0, POWER3, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();

                CurrentState = state.ForwardsToWall;
                break;
            case ForwardsToWall:
                RecentState = state.ForwardsToWall;
                //Forwards
                resetEncoder(leftFrontWheel);
                waitForMS(TIME_WAIT_SMALL);
                while (Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE4) {
                    arcade(POWER4, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();
                CurrentState = state.AlignAgainstWall;
                break;

            case AlignAgainstWall:
                RecentState = state.AlignAgainstWall;
                //TURN BACK
                while (gyroSensor.getIntegratedZValue() > GYRO5) {
                    telemetry.addData("Gyro", gyroSensor.getIntegratedZValue());
                    telemetry.addData("G Targ", GYRO5);
                    telemetry.update();
                    arcade(0, 0, POWER5, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();

                //strafe to wall
                waitForMS(TIME_WAIT_MEDIUM);
                while (range.getDistance(DistanceUnit.CM) > CM_FROM_WALL_VALUE) {
                    telemetry.addData("Dist", range.getDistance(DistanceUnit.CM));
                    telemetry.addData("D Targ", CM_FROM_WALL_VALUE);
                    telemetry.update();
                    arcade(0, POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

                }
                stopMotors();

                resetEncoder(leftFrontWheel);
                waitForMS(TIME_WAIT_SMALL);
                while (Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE6) {
                    arcade(0, POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

                }
                stopMotors();
                resetEncoder(leftFrontWheel);
                waitForMS(TIME_WAIT_SMALL);
                while (Math.abs(leftFrontWheel.getCurrentPosition()) < (DISTANCE6*.75)) {
                    arcade(0, -POWER6, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();

                CurrentState = state.GoingToBeacon1;
                break;

            case GoingToBeacon1:
                RecentState = state.GoingToBeacon1;
                //backwards to back color voltage_sensor
                while ((colorSensorLeftBottom.alpha() < COLOR_READING_FOR_LINE) && (colorSensorRightBottom.alpha() < COLOR_READING_FOR_LINE)) {
                    telemetry.addData("Made it", "Hooray!");
                    double lPower = POWER7 + (gyroSensor.getIntegratedZValue() / 50);
                    double rPower = POWER7 - (gyroSensor.getIntegratedZValue() / 50);
                    lPower = POWER7;
                    rPower = POWER7;
                    leftBackWheel.setPower(lPower);
                    leftFrontWheel.setPower(lPower);
                    rightBackWheel.setPower(rPower);
                    rightFrontWheel.setPower(rPower);
                    telemetry.addData("Gyro", gyroSensor.getIntegratedZValue());
                    telemetry.update();
                }
                stopMotors();

                CurrentState = state.PressingBeacon;








                break;
            case GoingToBeacon2:
                RecentState = state.GoingToBeacon2;

                while (Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE8) {
                    arcade(POWER8, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

                }
                waitForMS(500);
                ///forwards to front color voltage_sensor
                while ((colorSensorLeftBottom.alpha() < COLOR_READING_FOR_LINE) && (colorSensorRightBottom.alpha() < COLOR_READING_FOR_LINE)) {
                    double lPower = POWER8 + (gyroSensor.getIntegratedZValue() / 100);
                    double rPower = POWER8 - (gyroSensor.getIntegratedZValue() / 100);
                    leftBackWheel.setPower(lPower);
                    leftFrontWheel.setPower(lPower);
                    rightBackWheel.setPower(rPower);
                    rightFrontWheel.setPower(rPower);
                    telemetry.addData("Gyro", gyroSensor.getIntegratedZValue());
                }
                CurrentState = state.PressingBeacon;
                break;
            case PressingBeacon:
                //Body
                dim.setLED(0, true);
                dim.setLED(1, true);

                while ((colorSensorLeftBottom.alpha() < COLOR_READING_FOR_LINE) &&  (colorSensorRightBottom.alpha() < COLOR_READING_FOR_LINE)){
                    arcade(-POWER_HANDLE_COLOR, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();
                waitForMS(TIME_WAIT_SMALL);

                while ((colorSensorLeftBottom.alpha() < COLOR_READING_FOR_LINE) && (colorSensorRightBottom.alpha() < COLOR_READING_FOR_LINE)){
                        leftFrontWheel.setPower(POWER_HANDLE_COLOR/1.5);
                        leftBackWheel.setPower(POWER_HANDLE_COLOR/1.5);
                        rightFrontWheel.setPower(POWER_HANDLE_COLOR/1.5);
                        rightBackWheel.setPower(POWER_HANDLE_COLOR/1.5);
                }
                stopMotors();
                waitForMS(TIME_WAIT_MEDIUM);
                if (colorSensorOnSide.blue() > colorSensorOnSide.red()) {
                    rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
                    leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
                } else {
                    rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
                    leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
                }

                dim.setLED(0, false);
                dim.setLED(1, false);
                waitForMS(TIME_WAIT_LARGE);
                dim.setLED(0, true);
                dim.setLED(1, true);

                rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
                leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);

                if (RecentState == state.GoingToBeacon1) {
                    CurrentState = state.GoingToBeacon2;
                } else {
                    CurrentState = state.CapBall;
                }
                RecentState = state.PressingBeacon;



                CurrentState = state.Finished;
                break;
            case CapBall:
                RecentState = state.CapBall;
                //TURN BACK
                while (gyroSensor.getIntegratedZValue() < GYRO9) {
                    arcade(0, 0, POWER9, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();

                //Back Up
                resetEncoder(leftFrontWheel);
                waitForMS(TIME_WAIT_SMALL);
                while (Math.abs(leftFrontWheel.getCurrentPosition()) < DISTANCE10) {
                    arcade(POWER10, 0, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
                }
                stopMotors();

                CurrentState = state.Finished;
                break;
            case Finished:
                super.stop();
                break;
            default:
                super.stop();
        } //Code Below this point will run on every cycle
        stopMotors();
        waitForMS(TIME_WAIT_MEDIUM);
        telemetry.clear();
        telemetry.addData("State", CurrentState);
        telemetry.addData("Recent State", RecentState);
        telemetry.addData("turnLeft Bottom", colorSensorLeftBottom.alpha());
        telemetry.addData("turnRight Bottom", colorSensorRightBottom.alpha());
        telemetry.addData("Side Color", colorSensorOnSide.red() > 2 || colorSensorOnSide.blue() > 2
                ? colorSensorOnSide.red() > colorSensorOnSide.blue() ? "Blue" : "Red" : "Not Sure");
        telemetry.update();
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
