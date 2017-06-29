/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode.Current.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Current.Robot;
import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

/**
 * Created by Ethan Schaffer on 10/31/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op Lift Only", group="TeleOp")
@Disabled
public class TeleOpLiftOnly extends OpMode {
    public double rpmTarget = 1700;
    public static final double LEFT_SERVO_OFF_VALUE = Robot.LEFT_SERVO_OFF_VALUE;
    public static final double LEFT_SERVO_ON_VALUE = Robot.LEFT_SERVO_ON_VALUE;
    public static final double RIGHT_SERVO_ON_VALUE = Robot.RIGHT_SERVO_ON_VALUE;
    public static final double RIGHT_SERVO_OFF_VALUE = Robot.RIGHT_SERVO_OFF_VALUE;
    double liftTime;

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
    public boolean RECENT_G1_A = false;
    public boolean driveModeFOD = false;
    public double timeIn = 0;

    public boolean RECENT_B_BUTTON_C1 = false;
    public boolean RECENT_X_BUTTON_C1 = false;
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
    public static final String BALLBLOCKNAME = "s"; //MO Port 4
    public static final double BALLBLOCKOPEN = .62, BALLBLOCKCLOSED = 0;
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
    final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
    public AHRS navX;

    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed, lift;
    Servo leftButtonPusher, rightButtonPusher, ballBlock, topCapHolder;
    int pastEnc1 = 0, pastEnc2 = 0;
    int deltaEnc1 = 0, deltaEnc2 = 0;
    public double SHOOTERMAXVALUE = 1;
    double DeltaAvg;

    public double lastTime, lastEnc1, percentError = 0, power;
    boolean capOn = false;

    double timeWait = .33; //in seconds
    /*
    * PWR  Made/Shot
    * .2   4   / 10
    * .33  9   / 10
    * .5   9   / 10
    *  1   10  / 10
    * */
    double target = rpmTarget* timeWait; //Target Rotations per second


    @Override
    public void init() {
        leftFrontWheel = hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(RIGHT2NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get(LIFTNAME);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        ballBlock = hardwareMap.servo.get(BALLBLOCKNAME);
        leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);
        topCapHolder = hardwareMap.servo.get("c");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlock.setPosition(BALLBLOCKCLOSED);
        double volts = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        if(volts > 13.3){
            power = 0.40;
        } else if(volts > 13.1){
            power = 0.50;
        } else if(volts > 12.9){
            power = 0.55;
        } else if(volts > 12.6){
            power = 0.60;
        } else if(volts > 12.3) {
            power = 0.70;
        } else if(volts > 12.0) {
            power = 0.80;
        } else {
            power = 0.90;
        }
        topCapHolder.setPosition(0);
        while(lift.getCurrentPosition()!=0){

        }
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(!RECENT_B_BUTTON_C1 && gamepad1.b && !gamepad1.start){
            rpmTarget+=50;
        } else if (!RECENT_X_BUTTON_C1 && gamepad1.x){
            rpmTarget -=50;
        }
        RECENT_X_BUTTON_C1 = gamepad1.x;
        RECENT_B_BUTTON_C1 = gamepad1.b;
        telemetry.addData("RPM Target", rpmTarget);
        telemetry.addData("Drive Mode", driveModeFOD ? "FOD" : "Arcade");
        telemetry.addData("Cap Holder", topCapHolder.getPosition());
        telemetry.addData("Lift", lift.getCurrentPosition());
        telemetry.update();
        if(gamepad2.y){
            topCapHolder.setPosition(topCapHolder.getPosition() + .01);
        } else if(gamepad2.a){
            topCapHolder.setPosition(topCapHolder.getPosition() - .01);
        }

        if(gamepad2.left_trigger > .75 && gamepad2.right_trigger > .75 && !capOn){
            lift.setTargetPosition(180);
            lift.setPower(.65);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            capOn = true;
            liftTime = getRuntime();
        }
/*
        if(lift.getPower() > 0){
            if(Math.abs(lift.getCurrentPosition()) > 240*.25){
                lift.setPower(0);
            }
        }
*/
        if(capOn){
            if(gamepad2.dpad_left){
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(.25);
            } else if(gamepad2.dpad_right){
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(-.25);
            } else {
                lift.setPower(0);
            }
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
        double inputC = Math.abs(gamepad1.right_stick_x)> ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x: 0;

        double BIGGERTRIGGER = gamepad1.left_trigger > gamepad1.right_trigger ? gamepad1.left_trigger : gamepad1.right_trigger;
        //Ternary, the larger trigger value is set to the value BIGGERTRIGGER

        if(BIGGERTRIGGER > TRIGGERTHRESHOLD){ //If we have enough pressure on a trigger
            if( (Math.abs(inputY) > Math.abs(inputX)) && (Math.abs(inputY) > Math.abs(inputC)) ){ //If our forwards motion is the largest motion vector
                inputY /= 5*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 5*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 5*BIGGERTRIGGER; //slow down our power inputs
            } else if( (Math.abs(inputC) > Math.abs(inputX)) && (Math.abs(inputC) > Math.abs(inputY)) ){ //and if our turing motion is the largest motion vector
                inputY /= 4*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 4*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 4*BIGGERTRIGGER; //slow down our power inputs
            } else if( (Math.abs(inputX) > Math.abs(inputY)) && (Math.abs(inputX) > Math.abs(inputC)) ){ //and if our strafing motion is the largest motion vector
                inputY /= 3*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 3*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 3*BIGGERTRIGGER; //slow down our power inputs
            }
        }
        //Use the larger trigger value to scale down the inputs.

        if(!RECENT_G1_A && gamepad1.a){
            driveModeFOD = !driveModeFOD;
        }
        RECENT_G1_A = gamepad1.a;
        if(driveModeFOD){
            fieldOriented(inputY, inputX, inputC, 0, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        } else {
            arcadeMecanum(inputY, inputX, inputC, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);
        }

        /*
          ___  ___ _ ____   _____
         / __|/ _ \ '__\ \ / / _ \
         \__ \  __/ |   \ V / (_) |
         |___/\___|_|    \_/ \___/
         */

        if(RECENT_X_BUTTON && !gamepad2.x)
            LEFTSERVOSTATE = (LEFTSERVOSTATE == SERVOSTATE.OFF ? SERVOSTATE.ON : SERVOSTATE.OFF);
        switch (RIGHTSERVOSTATE){
            case ON:
                rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE); break;
            case OFF:
                rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE); break;
        }
        RECENT_X_BUTTON = gamepad2.x;

        if(RECENT_B_BUTTON && !gamepad2.b && !gamepad2.start)
            RIGHTSERVOSTATE = (RIGHTSERVOSTATE== SERVOSTATE.OFF ? SERVOSTATE.ON : SERVOSTATE.OFF);
        switch (LEFTSERVOSTATE){
            case ON:
                leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE); break;
            case OFF:
                leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE); break;
        }
        RECENT_B_BUTTON = gamepad2.b;

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
    public void fieldOriented(double y, double x, double c, double gyroheading, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double cosA = Math.cos(Math.toRadians(normalize(gyroheading)));
        double sinA = Math.sin(Math.toRadians(normalize(gyroheading)));
        double xOut = x * cosA - y * sinA;
        double yOut = x * sinA + y * cosA;
        arcadeMecanum(yOut, xOut, c, leftFront, rightFront, leftBack, rightBack);
    }

    public double normalize(double heading){
        //This allows us to normalize our motion to assume a gyro drift of 5 degrees or more.
        if (heading < 0) {
            return 360 - (Math.abs(heading) % 360);
        } else {
            return (heading % 360);
        }
    }
}
