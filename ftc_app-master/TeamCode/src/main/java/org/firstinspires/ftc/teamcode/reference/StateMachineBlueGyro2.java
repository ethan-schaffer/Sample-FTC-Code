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

@Autonomous(name="StateBlueGyro2", group="Autonomous")
@Disabled
public class StateMachineBlueGyro2 extends LinearOpMode {
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
    public static final String COLORSIDENAME = "cs"; //Port 2
    public static final String COLORLEFTBOTTOMNAME = "cb";//Port 3
    public static final String COLORRIGHTBOTTOMNAME = "cb2"; //Port 4

    public static final double LEFT_SERVO_OFF_VALUE = .20;
    public static final double LEFT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_OFF_VALUE = .20;

    private double ticksPerRev = 7;
    private double gearBoxOne = 40.0;
    private double gearBoxTwo = 24.0 / 16.0;
    private double gearBoxThree = 1.0;
    private double wheelDiameter = 4.0 * Math.PI;
    private double cmPerInch = 2.54;
    private double width = 31.75;

    public double cmPerTick = (wheelDiameter / (ticksPerRev * gearBoxOne * gearBoxTwo * gearBoxThree)) * cmPerInch; //Allows us to drive our roobt with accuracy to the centiment

    public enum states {
        Move, Shoot, TurnLeft, TurnRight, TurnLeftEnc, TurnRightEnc, StrafeLeft, StrafeRight, StrafeToWall, LineSearch, PressBeacon, Wait5Seconds, Finished
    }
    public team Alliance = team.Blue; //Sets our alliance color

    //The state object lets us declare all of our logic above.
    public class state{
        private states sName = states.Finished;
        private double sensorValue = 0;
        private double powerValue = 0;
        private team alliance;
        state(states s, double firstV, double secondV){
            sName = s;
            sensorValue = firstV;
            powerValue = secondV;
        }
        state(states s, double value){
            sName = s;
            sensorValue = value;
        }
        state(states s){
            sName = s;
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
    double tolerance = -5;
    //Editing this array would change how to auto runs, in it's entirety. If we could have a GUI to change these values from the phone, we could basically do doodle.
    state[] stateOrder = new state[]{
            //              State         Sensor       Power
            new state(states.Move,          60,     - 1.00),
            new state(states.TurnRightEnc,   60,       0.25),
            new state(states.Move,          230,    - 1.00),

            new state(states.TurnLeft,   - 25,       0.15),

            new state(states.StrafeToWall,  13,       0.13),

            new state(states.TurnLeft,      tolerance,        0.05),
            new state(states.TurnRight,     -tolerance,       0.05),

//            new state(states.LineSearch,    2,         0.10),

            new state(states.LineSearch,    2,         0.10),

            new state(states.StrafeToWall,  8,         0.10),

            new state(states.LineSearch,    2,       - 0.10),

            new state(states.PressBeacon,   team.Blue       ),

            new state(states.StrafeRight,   0.45,       0.75), //AWAY from wall

            new state(states.Move,          125,     - 0.50),

            new state(states.TurnRight,    -tolerance,         0.05), //in case we overshoot
            new state(states.TurnLeft,     tolerance,         0.05),

            new state(states.LineSearch,    2,       - 0.10),

            new state(states.StrafeToWall,  11,         0.13),

            new state(states.LineSearch,    2,        0.10),

            new state(states.StrafeToWall,  8,         0.10),

            new state(states.PressBeacon,   team.Blue       ),
            new state(states.StrafeRight,   0.25,      1.00), //Away from wall

            new state(states.TurnRightEnc,   45,       1.00),

            new state(states.Move,          225,        1.00),
        };

    //NotSensed is for the Color Sensor while we are pushing the beacon.
    public enum team {
        Red, Blue, NotSensed
    }



    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed;
    Servo leftButtonPusher, rightButtonPusher, ballBlockRight, ballBlockLeft;
    ColorSensor colorSensorLeftBottom, colorSensorOnSide;
    ModernRoboticsI2cRangeSensor range;
    ModernRoboticsI2cGyro gyroSensor;
    DeviceInterfaceModule dim;
    state CurrentState;
    long lastTime;
    long time;
    boolean initialized = false, movedServo = false;
    double circleFrac, movement, changeFactor, modified, currentDistance;
    team colorReading = team.NotSensed;
    int redReading, blueReading;

    public int stateNumber = -1;
    public state updateState() throws InterruptedException {
        stopMotors();
        idle();
        stateNumber++;
        colorReading = team.NotSensed;
        initialized = false;
        movedServo = false;
        time = 0;
        idle();
        if(stateNumber == stateOrder.length){
            return new state(states.Finished);
        }
        return stateOrder[stateNumber];
    }
    public state getCurrentState(){
        return stateOrder[stateNumber];
    }

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
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRONAME);

        gyroSensor.calibrate();
        int timer = 0;
        int cycler = 0;
        while(gyroSensor.isCalibrating()) {
            timer++;
            if(timer % 10 == 0){
                cycler++;
            }
            if (cycler == 1) {
                telemetry.addData("Gyro", " is Calibrating.");
            } else if (cycler == 2){
                telemetry.addData("Gyro", " is Calibrating..");
            } else {
                cycler = 0;
                telemetry.addData("Gyro", " is Calibrating...");
            }
            telemetry.update();
        }
        telemetry.addData("Gyro", "Calibrated (" + timer + " cycles)");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        colorSensorLeftBottom = hardwareMap.colorSensor.get(COLORLEFTBOTTOMNAME);
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorLeftBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");

        telemetry.addData("Raw Ultrasonic", range.rawUltrasonic());
        telemetry.addData("Color Side Red", colorSensorOnSide.red());
        telemetry.addData("Color turnLeft Alpha", colorSensorLeftBottom.alpha());

        telemetry.update();
        CurrentState = updateState();
        colorSensorLeftBottom.enableLed(true);

        waitForStart();
        double totalTime = 0;
        double cm, ticks, degrees, power, Seconds, Speed, Time, targetPosition, diffVal, gyroThreshold = 1;
        int RBPos, RFPos, LBPos, LFPos, Average;
        resetEncoder(leftFrontWheel);
        while((CurrentState.getState() != states.Finished) && (opModeIsActive()) && (totalTime < (30*1000))) {
            totalTime++; //Because every cycle takes exactly 1ms, we know how long we have been running. totalTime will automatically stop the robot at 30 seconds
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
                    if(time < 50){
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
                        dim.setLED(0, false); //Red
                        dim.setLED(1, true); //Blue
                        if(!movedServo){
                            leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
                            movedServo = true;
                        }
                        if(time > 1250){
                            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
                            CurrentState = updateState();
                        }
                        //The left servo is below the voltage_sensor
                    } else {
                        dim.setLED(0, false); //Red
                        dim.setLED(1, true); //Blue
                        if(!movedServo) {
                            rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
                            movedServo = true;
                        }
                        if(time > 1250){
                            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
                            CurrentState = updateState();
                        }
                    }
                    break;
                case Finished:
                    telemetry.clear();
                    telemetry.addLine("All Done!");
                    telemetry.addLine("It look " + totalTime/1000 + " seconds to complete this autonomous");
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
            telemetry.addData("Uptime", (totalTime/1000)+" seconds");

            telemetry.addData("turnLeft Bottom", colorSensorLeftBottom.alpha());
            telemetry.addData("Side Color", colorReading);
            telemetry.addData("G", gyroSensor.getHeading());
            telemetry.update();
            while(1000*1000 > SystemClock.elapsedRealtimeNanos() - lastTime){

            } //guarantees a loop speed of 1000 milliseconds.
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


}
