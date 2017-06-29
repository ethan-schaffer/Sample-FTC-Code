/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode.TeleOpSetPowers;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Created by Ethan Schaffer on 10/31/2016.
 */
@TeleOp(name=" * Tele Op 60 *", group="TeleOpOld")
@Disabled
public class TeleOpFinal60 extends OpMode {
    public static final double SHOOTERMAXVALUE = .60;

    //TWEAKING VALUES
    public static final double LEFT_SERVO_OFF_VALUE = .3;
    public static final double LEFT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_OFF_VALUE = .3;

    public static final double MAXINFEEDPOWER = 1;
    public static final double MAXOUTFEEDPOWER = -1;

    //GOOD VALUES
    public enum INFEEDSTATE {
        IN, OUT, NOTGOING
    }
    public enum SHOOTERSTATE {
        SHOOTING, BACK, NOTSHOOTING
    }
    public enum SERVOSTATE {
        ON, OFF
    }
    public INFEEDSTATE INFEEDSTATUS = INFEEDSTATE.NOTGOING;
    public SHOOTERSTATE SHOOTERSTATUS = SHOOTERSTATE.NOTSHOOTING;
    public SERVOSTATE RIGHTSERVOSTATE = SERVOSTATE.OFF;
    public SERVOSTATE LEFTSERVOSTATE = SERVOSTATE.OFF;

    public boolean RECENT_DRIVER_RIGHT_BUMPER = false;
    public boolean RECENT_Y_BUTTON = false;
    public boolean RECENT_TRIGGER = false;

    public boolean RECENT_X_BUTTON = false;
    public boolean RECENT_B_BUTTON = false;
    public boolean RECENT_TOPHAT_DOWN = false;
    public boolean RECENT_TOPHAT_UP = false;

    public boolean RECENT_LB = false;
    public boolean RECENT_RB = false;


    public static final double SLOWDOWNVALUE = 5;
    public static final double TRIGGERTHRESHOLD = .2;
    public static final double ACCEPTINPUTTHRESHOLD = .15;
    public static final double SCALEDPOWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train

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
    public static final String LIFTNAME = "l"; //2S Port 1
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 1
    public static final String COLORLEFTBOTTOMNAME = "cb";//Port 2
    public static final String COLORRIGHTBOTTOMNAME = "cb2"; //Port 4
    public static final String GYRONAME = "g"; //Port 4

    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed, lift;
    Servo leftButtonPusher, rightButtonPusher, ballBlockRight, ballBlockLeft;
    ColorSensor colorSensorOnSide, colorSensorLeftBottom, colorSensorRightBottom;
    ModernRoboticsI2cGyro gyroSensor;
    DeviceInterfaceModule dim;
    ModernRoboticsI2cRangeSensor range;

    @Override
    public void init() {
        leftFrontWheel = hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(RIGHT2NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get(LIFTNAME);

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
        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
//        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        /*
              _                 _
             | |               | |
          ___| |__   ___   ___ | |_
         / __| '_ \ / _ \ / _ \| __|
         \__ \ | | | (_) | (_) | |_
         |___/_| |_|\___/ \___/ \__|
        */
        if(RECENT_LB && !gamepad2.left_bumper){
            SHOOTERSTATUS =(SHOOTERSTATUS == SHOOTERSTATE.SHOOTING) ? SHOOTERSTATE.NOTSHOOTING : SHOOTERSTATE.SHOOTING;
        } else if (RECENT_RB && !gamepad2.right_bumper){
            SHOOTERSTATUS =(SHOOTERSTATUS == SHOOTERSTATE.BACK) ? SHOOTERSTATE.NOTSHOOTING : SHOOTERSTATE.BACK;
        }
        //Ternary Operations used to toggle SHOOTERSTATUS
        switch(SHOOTERSTATUS){
            case SHOOTING:
                shoot1.setPower(SHOOTERMAXVALUE);
                shoot2.setPower(SHOOTERMAXVALUE);
                ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
                ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
                break;
            case BACK:
                shoot1.setPower(-SHOOTERMAXVALUE);
                shoot2.setPower(-SHOOTERMAXVALUE);
                ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
                ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
                break;
            default:
                shoot1.setPower(0);
                shoot2.setPower(0);
                ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
                ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        }
        RECENT_LB = gamepad2.left_bumper;
        RECENT_RB = gamepad2.right_bumper;
        /*
          _        __              _
         (_)      / _|            | |
          _ _ __ | |_ ___  ___  __| |
         | | '_ \|  _/ _ \/ _ \/ _` |
         | | | | | ||  __/  __/ (_| |
         |_|_| |_|_| \___|\___|\__,_|
        */
        if(RECENT_TOPHAT_DOWN && !gamepad2.dpad_down){
            INFEEDSTATUS = (INFEEDSTATUS == INFEEDSTATE.IN ? INFEEDSTATE.NOTGOING : INFEEDSTATE.IN);
        }
        else if(RECENT_TOPHAT_UP && !gamepad2.dpad_up){
            INFEEDSTATUS = (INFEEDSTATUS == INFEEDSTATE.OUT ? INFEEDSTATE.NOTGOING : INFEEDSTATE.OUT);
        }
        if(gamepad2.left_stick_y > .5){
            INFEEDSTATUS = INFEEDSTATE.IN;
        } else if(gamepad2.left_stick_y < -.5){
            INFEEDSTATUS = INFEEDSTATE.OUT;
        } else if(INFEEDSTATUS == INFEEDSTATE.NOTGOING){
            INFEEDSTATUS = INFEEDSTATE.NOTGOING;
        }
        RECENT_TOPHAT_DOWN = gamepad2.dpad_down;
        RECENT_TOPHAT_UP = gamepad2.dpad_up;
/*
        switch (INFEEDSTATUS){
            case IN:
                infeed.setPower(MAXINFEEDPOWER);
                break;
            case OUT:
                infeed.setPower(MAXOUTFEEDPOWER);
                break;
            case NOTGOING:
                infeed.setPower(0);
                break;
        }
*/
        if(gamepad2.dpad_down){
            infeed.setPower(MAXINFEEDPOWER);
        } else if(gamepad2.dpad_up) {
            infeed.setPower(MAXOUTFEEDPOWER);
        } else {
            infeed.setPower(0);
        }
        /*
              _      _
             | |    (_)
           __| |_ __ ___   _____
          / _` | '__| \ \ / / _ \
         | (_| | |  | |\ V /  __/
          \__,_|_|  |_| \_/ \___|
         */
        double inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y : 0;
        double inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x : 0;
        double inputC = Math.abs(gamepad1.right_stick_x)> ACCEPTINPUTTHRESHOLD ? gamepad1.right_stick_x: 0;

        double BIGGERTRIGGER = gamepad1.left_trigger > gamepad1.right_trigger ? gamepad1.left_trigger : gamepad1.right_trigger;
        //Ternary, the larger trigger value is set to the value BIGGERTRIGGER

        if(BIGGERTRIGGER > TRIGGERTHRESHOLD){ //If we have enough pressure on a trigger
            if( (Math.abs(inputY) > Math.abs(inputX)) && (Math.abs(inputY) > Math.abs(inputC)) ){ //If our forwards motion is the largest motion vector
                inputY /= 5*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 5*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 5*BIGGERTRIGGER; //slow down our power inputs
            }
            if( (Math.abs(inputC) > Math.abs(inputX)) && (Math.abs(inputC) > Math.abs(inputY)) ){ //and if our turing motion is the largest motion vector
                inputY /= 4*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 4*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 4*BIGGERTRIGGER; //slow down our power inputs
            }
            if( (Math.abs(inputX) > Math.abs(inputY)) && (Math.abs(inputX) > Math.abs(inputC)) ){ //and if our strafing motion is the largest motion vector
                inputY /= 3*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 3*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 3*BIGGERTRIGGER; //slow down our power inputs
            }
        }
        //Use the larger trigger value to scale down the inputs.

        if(!gamepad1.right_bumper && RECENT_DRIVER_RIGHT_BUMPER){
            if(leftBackWheel.getDirection() == DcMotorSimple.Direction.FORWARD){
                leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                leftBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                leftFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
                rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
        if(leftBackWheel.getDirection() == DcMotorSimple.Direction.REVERSE){
            inputC *= -1;
        }

        RECENT_DRIVER_RIGHT_BUMPER = gamepad1.right_bumper;
        // The above code will reverse the motors (and therefore the robot's orientation)
        // When the driver presses the 'A' button

        arcadeMecanum(inputY, inputX, inputC, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        /*
          ___  ___ _ ____   _____
         / __|/ _ \ '__\ \ / / _ \
         \__ \  __/ |   \ V / (_) |
         |___/\___|_|    \_/ \___/
         */

        if(RECENT_X_BUTTON && !gamepad2.x){
            LEFTSERVOSTATE = (LEFTSERVOSTATE == SERVOSTATE.OFF ? SERVOSTATE.ON : SERVOSTATE.OFF);
        }
        switch (RIGHTSERVOSTATE){
            case ON:
                leftButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
                break;
            case OFF:
                leftButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        }
        RECENT_X_BUTTON = gamepad2.x;

        if(RECENT_B_BUTTON && !gamepad2.b){
            RIGHTSERVOSTATE = (RIGHTSERVOSTATE== SERVOSTATE.OFF ? SERVOSTATE.ON : SERVOSTATE.OFF);
        }
        switch (LEFTSERVOSTATE){
            case ON:
                rightButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
                break;
            case OFF:
                rightButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        }
        RECENT_B_BUTTON = gamepad2.b;

        if(gamepad2.a){
            lift.setPower(1);
        } else if(gamepad2.y){
            lift.setPower(-.25);
        } else {
            lift.setPower(0);
        }


        /*
          _       _                     _
         | |     | |                   | |
         | |_ ___| | ___ _ __ ___   ___| |_ _ __ _   _
         | __/ _ \ |/ _ \ '_ ` _ \ / _ \ __| '__| | | |
         | ||  __/ |  __/ | | | | |  __/ |_| |  | |_| |
          \__\___|_|\___|_| |_| |_|\___|\__|_|   \__, |
                                                  __/ |
                                                 |___/
        */
        telemetry.addData("LF", leftFrontWheel.getPower());
        telemetry.addData("LB", leftBackWheel.getPower());
        telemetry.addData("RF", rightFrontWheel.getPower());
        telemetry.addData("RB", rightBackWheel.getPower());

        telemetry.addData("Infeed", infeed.getPower() > .1 ? "IN" : infeed.getPower() < -.1 ? "OUT" : "OFF");
        telemetry.addData("Shooter", SHOOTERSTATUS == SHOOTERSTATE.SHOOTING ? "Shooting" : "Not Shooting");
        telemetry.addData("turnRight Servo", RIGHTSERVOSTATE);
        telemetry.addData("turnLeft Servo", LEFTSERVOSTATE);
        //Ternary, basically it just outputs the Infeed states.

        telemetry.update();
    }

    // y - forwards
    // x - side
    // c - rotation
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
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
        double scaledPower = SCALEDPOWER;

        leftFront.setPower(leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower));
        rightFront.setPower(rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower));
        leftBack.setPower(leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower));
        rightBack.setPower(rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower));
    }
}
