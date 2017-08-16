package org.firstinspires.ftc.teamcode.MecanumSamples;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

import java.util.Arrays;
import java.util.Objects;

/**
 * Created by Ethan Schaffer
 */

public class MecanumChassis {
    public enum team {
        Red, Blue, NotSensed
    }

    private static final String DIMNAME = "dim"; //second DIM, reserved for NavX
    private LinearOpMode l;
    private boolean initialized = false;
    private final double ticksPerRev = 7;
    private final double gearBoxOne = 40.0;
    private final double gearBoxTwo = 24.0 / 16.0;
    private final double gearBoxThree = 1.0;
    private final double wheelDiameter = 4.0 * Math.PI;
    private final double cmPerInch = 2.54;
    private final double cmPerTick = (wheelDiameter / (ticksPerRev * gearBoxOne * gearBoxTwo * gearBoxThree)) * cmPerInch;
    //Allows us to drive our robot with accuracy to the centimeter

    private final double width = 31.75;
    // The wheel's width is used to calculate turn distance by Encoder ticks, using our turnLeftEnc method.

    private static final String COLORSIDENAME = "cs"; //Port 1
    private VoltageSensor voltageGetter;

    private DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    private ColorSensor colorSensorOnSide, colorSensorBottom;
    public ModernRoboticsI2cGyro gyroSensor;
    private DeviceInterfaceModule dim;
    public ModernRoboticsI2cRangeSensor range;
    private AHRS navX;
    private final double distance = 2000 / (172 * cmPerInch);
    Telemetry t;

    public void sensorsInfo() {
        try {
            t.addData("NavX", navX.isConnected() ? navX.getYaw() : "Disconnected");
            t.addData("Color Side Red", colorSensorOnSide.red());
            t.addData("Color Side Blue", colorSensorOnSide.blue());
            t.addData("Range CM", range.getDistance(DistanceUnit.CM));
        } catch (NullPointerException e) {
            t.addData("NavX", "Disabled!");
        }
        try {
            t.addData("Bottom", colorSensorBottom.alpha());
        } catch (Exception e) {
            t.addData("Bottom", "None");
        }
        /*
        t.addData("LF", leftFrontWheel.getPower());
        t.addData("LB", leftBackWheel.getPower());
        t.addData("RF", rightFrontWheel.getPower());
        t.addData("RB", rightBackWheel.getPower());
        */
        t.update();
    }

    // We initialize all of our Motors and Servos locally. This helps our other files maintain readability,
    // and allows us to change how the hardware map works here or otherwise.
    public void initialize(LinearOpMode lInput, HardwareMap hardwareMap, Telemetry telemetry, boolean navXOn) {
        l = lInput;
        initialized = true;
        t = telemetry;
        leftFrontWheel = hardwareMap.dcMotor.get(UniversalConstants.LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(UniversalConstants.LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(UniversalConstants.RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(UniversalConstants.RIGHT2NAME);
        rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageGetter = hardwareMap.voltageSensor.get("Motor Controller 1");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");

        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorOnSide.enableLed(false);

        dim = hardwareMap.get(DeviceInterfaceModule.class, DIMNAME);
        // If the navXOn boolean is input as false, we can skip this step.
        // This will only be the case in a fraction of our routes,
        // but it is a good fallback if the navX is broken.
        if (navXOn) {
            navX = new AHRS(dim, hardwareMap.i2cDevice.get("n").getPort(), AHRS.DeviceDataType.kProcessedData, (byte) 50);
            navX.zeroYaw();
            while (navX.isCalibrating() && !l.isStopRequested()) {
                telemetry.addData("Gyro", "Calibrating");
                telemetry.addData("Yaw", navX.getYaw());
                if (!navX.isConnected()) {
                    telemetry.addData("NavX", "DISCONNECTED!");
                } else {
                    telemetry.addData("NavX", "Connected!");
                }
                telemetry.update();
            }
            // Allow the NavX to calibrate before exiting the
            telemetry.addData("Yaw", navX.getYaw());
            if (!navX.isConnected()) {
                telemetry.addData("NavX", "DISCONNECTED!");
            } else {
                telemetry.addData("NavX", "Connected!");
            }
            telemetry.update();
        } else {
            telemetry.addData("NavX", "None");
        }

        //We found that if you don't set the LED until after the waitForStart(), it doesn't work as well.
        colorSensorOnSide.enableLed(false);
    }

    // This function resets the encoders of the 4 drive wheels.
    // We call it at the start of each method that uses drive encoders,
    // to ensure that our encoders are accurate and in the correct mode.
    public void ResetDriveEncoders() throws InterruptedException {
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (leftFrontWheel.getCurrentPosition() != 0 || leftBackWheel.getCurrentPosition() != 0
                || rightBackWheel.getCurrentPosition() != 0 || rightFrontWheel.getCurrentPosition() != 0) {
            l.idle();
        }
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // This method lets us set strafing power to the motors
    // such that we strafe either left or right.
    public void SetStrafePower(String Direction, double Speed) {
        if (Objects.equals(Direction.toLowerCase(), "Left".toLowerCase())) {
            rightFrontWheel.setPower(Speed);
            rightBackWheel.setPower(-Speed);
            leftFrontWheel.setPower(-Speed);
            leftBackWheel.setPower(Speed);
        }
        if (Objects.equals(Direction.toLowerCase(), "Right".toLowerCase())) {
            rightFrontWheel.setPower(-Speed);
            rightBackWheel.setPower(Speed);
            leftFrontWheel.setPower(Speed);
            leftBackWheel.setPower(-Speed);
        }
    }

    // This is our most used, and most useful method.
    // We use it to turn to within our  threshold angle of 0.
    // The default power is .05 as that power performed with the best accuracy in our testing.
    public void AlignToWithin(double threshold) {
        AlignToWithin(threshold, .05);
    }

    // The align to within algorithm uses our team's unique method of non-recovery turning.
    // Because the robot exits the control loop once the reading is past the target,
    // we can apply this logic and use it to control our turning to a precise degree of accuracy.
    public void AlignToWithin(double threshold, double power) {
        TurnRightAbsolute(-threshold, power);
        TurnLeftAbsolute(threshold, power);
        TurnRightAbsolute(-threshold, power);
    }

    // The AlignToWithinOf method aligns to within threshold degrees of expected.
    // To do this, we subtract or add threshold to the expected reading,
    // following a similar structure to that of AlignToWithin
    public void AlignToWithinOf(double expected, double threshold, double power) {
        TurnRightAbsolute(expected - threshold, power);
        TurnLeftAbsolute(expected + threshold, power);
        TurnRightAbsolute(expected - threshold, power);
    }

    // Sets the same power to all four wheels. This lets us stop and start the robot easily,
    // and in a more _ way.
    public void SetDrivePower(double power) {
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
    }

    // Using the encoders, we move to a certain distance.
    // If ShooterOn is true, we will handle RPM while moving.
    // The
    public void Move(double centimeters, double power) throws InterruptedException {
        if (!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double ticks = centimeters / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        while (avg < ticks && l.opModeIsActive()) {
            SetDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        SetDrivePower(0);
    }

    //Follows the same logic as Move.
    // However, it makes movements based on the change in encoders,
    // rather than by resetting the encoders.
    // This allows us to call MoveByDelta right after some other method that has the robot drive.
    // Thanks to this, we can achieve more fluid motion while we are midway through a movement.
    public void MoveByDelta(double centimeters, double power) {
        if (!l.opModeIsActive())
            Finish();
        int StartRBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int StartRFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int StartLBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int StartLFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (StartRBPos + StartLFPos + StartLBPos + StartRFPos) / 4;
        double ticks;
        if (power > 0) {
            ticks = avg + Math.abs(centimeters);
        } else {
            ticks = avg - Math.abs(centimeters);
        }
        int RBPos, LFPos, LBPos, RFPos;
        while (avg < ticks && l.opModeIsActive()) {
            SetDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = Math.abs(RBPos - StartRBPos) + Math.abs(RFPos - StartRFPos) + Math.abs(LBPos - StartLBPos) + Math.abs(LFPos - StartLFPos);
            avg /= 4;
        }
        SetDrivePower(0);
    }

    // Drive in a direction until the robot's pitch changes.
    // This is useful for parking on the ramp structure.
    public void MoveToPitch(double pitch, double power) {
        while (l.opModeIsActive() && (Math.abs(navX.getPitch()) < pitch)) {
            SetDrivePower(power);
        }
        SetDrivePower(0);
    }

    // MoveCoast functions like Move, but the drive wheels are not stopped at the end.
    // This allows for more fluid motion while driving autonomously,
    // especially on routes like our main blue one where we are aiming for
    // more fluid motion.
    public void MoveCoast(double sensor, double power) throws InterruptedException {
        if (!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double ticks = sensor / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        while (avg < ticks && l.opModeIsActive()) {
            SetDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
    }

    // The ForwardsPLoop method slows the robot down
    // to give us more control of our distance travelled.
    // We use it when we need to be especially precise with our lineup,
    // such as when we need to score particles into the center vortex.
    public void ForwardsPLoop(double sensor, double maxPower) throws InterruptedException {
        //Max Power should be normally set to 1, but for very precise Movements a value of .diagonalBrokeThreshold or lower is recommended.
        if (!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double ticks = sensor / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        double power;
        while (avg < ticks && l.opModeIsActive()) {
            power = Range.clip((ticks - avg) / ticks, .1, Math.abs(maxPower));
            SetDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        SetDrivePower(0);
    }

    // The default max power is 1.0, or 100%
    public void ForwardsPLoop(double sensor) throws InterruptedException {
        ForwardsPLoop(sensor, 1.0);
    }

    //Same as forwardsPLoop, but backwards.
    public void BackwardsPLoop(double sensor, double maxPower) throws InterruptedException {
        //Max Power should be normally set to 1, but for very precise Movements a value of .diagonalBrokeThreshold or lower is recommended.
        if (!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double ticks = Math.abs(sensor) / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        double power;
        while (avg < ticks && l.opModeIsActive()) {
            power = -Range.clip((ticks - avg) / ticks, .1, Math.abs(maxPower));
            SetDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        SetDrivePower(0);
    }

    // The default max power is 1.0, or 100%
    public void BackwardsPLoop(double sensor) throws InterruptedException {
        BackwardsPLoop(sensor, 1.0);
    }

    // TurnLeftPLoop is designed to use a P style control loop to
    // turn a bit faster.
    // It has yet to be fully tested.
    public void TurnLeftPLoop(double degrees, double maxPower) {
        //Max Power should be normally set to .5, but for very precise turns a value of .05 is recommended.
        if (!l.opModeIsActive())
            Finish();
        double power;
        while (navX.getYaw() >= degrees && l.opModeIsActive()) {
            power = Range.clip((navX.getYaw() - degrees) / degrees, .05, maxPower);
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        SetDrivePower(0);
    }

    // Turns left to a precise angle, regardless of current lineup.
    public void TurnLeftAbsolute(double degrees, double power) {
        if (!l.opModeIsActive())
            Finish();
        while (navX.getYaw() >= degrees && l.opModeIsActive()) {
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        SetDrivePower(0);

    }

    // Turns left to an angle based on the current reading.
    // This is most useful when we are using our file reading code,
    // or when we are later into the route and are worried the navX
    // may be drifted.
    public void TurnLeftRelative(double degrees, double power) throws InterruptedException {
        if (!navX.isConnected()) {
            TurnLeftEnc(degrees, .10);
            return;
        }
        if (!l.opModeIsActive())
            Finish();
        double yaw = navX.getYaw();
        degrees = -Math.abs(degrees);
        while (Math.abs(yaw - navX.getYaw()) < degrees && l.opModeIsActive()) {
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        SetDrivePower(0);

    }

    // Functions just like TurnLeftAbsolute,
    // but allows us to input a positive number,
    // thus improving code legibility.
    public void TurnLeft(double degrees, double power) {
        TurnLeftAbsolute(-Math.abs(degrees), power);
    }

    // Uses Motor Encoders to turn faster, without the reliance on a gyroscope.
    // This turning tends to be less accurate,
    // but is good for faster turns that we are making
    // towards the end of a route.
    // It is also good as a fallback if the navX breaks or disconnects.
    public void TurnLeftEnc(double degrees, double power) throws InterruptedException {
        if (!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double changeFactor = 90;
        double modified = changeFactor + degrees;

        double circleFrac = modified / 360;
        double cm = width * circleFrac * Math.PI * 2;
        double movement = cm / cmPerTick;
        double ticks = movement / 2;

        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
        leftBackWheel.setPower(-power);
        leftFrontWheel.setPower(-power);

        double avg;
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos) / 4;
        } while (avg < ticks && l.opModeIsActive());
        SetDrivePower(0);
    }

    /*
    *
    * For info on how any of these methods work, please refer to the documentation of the matching method above.
    *
    * */
    public void TurnRightPLoop(double degrees, double maxPower) {
        //Max Power should be normally set to .5, but for very precise turns a value of .05 is recommended.
        if (!l.opModeIsActive())
            Finish();
        double power;
        while (navX.getYaw() <= degrees && l.opModeIsActive()) {
            power = Range.clip((degrees - navX.getYaw()) / degrees, .05, maxPower);
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        SetDrivePower(0);
    }

    public void TurnRightAbsolute(double degrees, double power) {
        if (!l.opModeIsActive())
            Finish();
        while (navX.getYaw() <= degrees && l.opModeIsActive()) {
            rightBackWheel.setPower(-power);
            rightFrontWheel.setPower(-power);
            leftBackWheel.setPower(power);
            leftFrontWheel.setPower(power);
        }
        SetDrivePower(0);
    }

    public void TurnRight(double degrees, double power) {
        TurnRightAbsolute(degrees, power);
    }

    public void TurnRightRelative(double degrees, double power) {
        if (!l.opModeIsActive())
            Finish();
        double yaw = navX.getYaw();
        while (Math.abs(yaw - navX.getYaw()) < degrees && l.opModeIsActive()) {
            rightBackWheel.setPower(-power);
            rightFrontWheel.setPower(-power);
            leftBackWheel.setPower(power);
            leftFrontWheel.setPower(power);
        }
        SetDrivePower(0);
    }

    public void TurnRightEnc(double degrees, double power) throws InterruptedException {
        if (!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double changeFactor = 90;
        double modified = changeFactor + degrees;

        double circleFrac = modified / 360;
        double cm = width * circleFrac * Math.PI * 2;
        double movement = cm / cmPerTick;
        double ticks = movement / 2;

        rightBackWheel.setPower(-power);
        rightFrontWheel.setPower(-power);
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);

        double avg;
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos) / 4;
        } while (avg < ticks && l.opModeIsActive());
        SetDrivePower(0);
    }

    // The StrafeLeft and StrafeRight methods allow us to strafe based on encoder readout.
    // However, they don't work very well, as Mecanum drive is inherently very low-torque,
    // and can strafe even further than expected.
    public void StrafeLeft(double centimeters, double power) {
        if (!l.opModeIsActive())
            Finish();
        double ticks = centimeters / cmPerTick;
        ticks *= distance;
        int avg;
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(-power);
        leftFrontWheel.setPower(-power);
        leftBackWheel.setPower(power);
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos) / 4;
        } while (avg < ticks && l.opModeIsActive());
        SetDrivePower(0);
    }

    public void StrafeRight(double centimeters, double power) {
        if (!l.opModeIsActive())
            Finish();
        double ticks = centimeters / cmPerTick;
        ticks *= distance;
        int avg;
        SetStrafePower("Right", power);
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos) / 4;
        } while (avg < ticks && l.opModeIsActive());
        SetDrivePower(0);
    }

    // The getRange method filters out bogus range values of 255.
    // We found that these values were common around other robots
    // using similar sensors, so wee filter them out.
    public double getRange(double previousReading) {
        double c = range.getDistance(DistanceUnit.CM);
        if (c == 255) {
            return previousReading;
        } else {
            return c;
        }
    }

    //
    public void StrafeToWall(double centimeters, double power) throws InterruptedException {
        double startYaw = navX.getYaw();
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        double rangeLastSecond = 254;
        double lastTime = l.getRuntime();
        while (pastRange > centimeters && l.opModeIsActive()) {
            if (l.getRuntime() - 1 >= lastTime) { //if a second has passed
                lastTime = l.getRuntime();
                if (rangeLastSecond == pastRange) {
                    StrafeFromWall(range.getDistance(DistanceUnit.CM) + 1, .10);
                    Move(15 * 2.54, .10);
                    AlignToWithinOf(startYaw, .5, .05);
                    StrafeToWall(centimeters, power);
                    Move(15 * 2.54, -.10);
                }
                rangeLastSecond = pastRange;
            }
            pastRange = getRange(pastRange);
            SetStrafePower("Left", power);
        }
        if (range.getDistance(DistanceUnit.CM) == 255) {
            StrafeToWall(centimeters, power);
        }
        SetDrivePower(0);
    }

    public void StrafeFromWall(double centimeters, double power) {
        double pastRange = 0;
        if (!l.opModeIsActive())
            Finish();
        while (pastRange < centimeters && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            SetStrafePower("Right", power);
        }
        if (range.getDistance(DistanceUnit.CM) == 255) {
            StrafeFromWall(centimeters, power);
        }
        SetDrivePower(0);
    }

    // Finds the white tape line beneath the robot.
    public void LineSearch(double power) {
        LineSearch(2, power);
    }

    // The sensor value is the expected value of
    // the Color Sensor's Alpha reading.
    public void LineSearch(double sensor, double power) {
        if (!l.opModeIsActive())
            Finish();
        while (colorSensorBottom.alpha() < sensor && l.opModeIsActive()) {
            SetDrivePower(power);
        }
        SetDrivePower(0);
    }

    // Follows the same logic as AlignToWithinOf,
    public void StrafeToPrecise(double cm, double threshold, double power) throws InterruptedException {
        StrafeToWall(cm + threshold, power);
        StrafeFromWall(cm - threshold, power);
        StrafeToWall(cm + threshold, power);
    }

    double getRobotVoltage() {
        return voltageGetter.getVoltage();
    }


    public void WaitForRange(double readingTarget) {
        double rangeR = 254;
        while (rangeR > readingTarget && l.opModeIsActive()) {
            rangeR = getRange(rangeR);
        }
    }

    // Runs the super.stop() method, and turns off all of the sensors
    public void Finish() {
        navX.close();
        colorSensorOnSide.close();
        colorSensorBottom.close();
        range.close();
        l.stop();
    }

    // If the NavX is offset by more than this angle,
    // the robot will exit the strafing control loop and
    // run the handleCollision code.
    double diagonalBrokeThreshold = 15;

    // We want to back away from whatever dislodged us,
    // and then strafe to the wall to the same distance as the
    // DiagonalForwardsLeft was told to.
    public void handleCollision(double sensor) throws InterruptedException {
        boolean turnedTooFarRight;
        if (navX.getYaw() < 0) {
            turnedTooFarRight = true;
            Move(15, 1.0);
        } else {
            turnedTooFarRight = false;
            Move(15, -1.0);
        }
        StrafeToWall(sensor, .20);
        AlignToWithin(1.5, .05);
        // Line back up to center.
        if (!turnedTooFarRight) {
            Move(25, 1.0);
        } else {
            Move(25, -1.0);
        }
        // This will give us a similar outcome to the expected outcome
        // of the call to DiagonalForwards
    }

    // DiagonalForwardsLeft will strafe at a diagonal until the
    public boolean DiagonalForwardsLeft(double distance, double power) throws InterruptedException {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        double angStart = navX.getYaw();
        while (pastRange > distance && l.opModeIsActive() && Math.abs(navX.getYaw() - angStart) < diagonalBrokeThreshold) {
            pastRange = getRange(pastRange);
            arcadeMecanum(1, -1, 0);
        }
        if (Math.abs(navX.getYaw() - angStart) > diagonalBrokeThreshold) {
            handleCollision(distance);
            SetDrivePower(0);
            return true;
        }
        SetDrivePower(0);
        return false;
    }

    // This method functions like DiagonalForwardsLeft,
    // but it takes separate input for the
    // x and y directions of motion.
    public boolean DiagonalForwardsLeft(double distance, double yIn, double xIn) throws InterruptedException {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();

        double angStart = navX.getYaw();
        while (pastRange > distance && Math.abs(navX.getYaw() - angStart) < diagonalBrokeThreshold && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(yIn, -xIn, 0);
        }
        if (Math.abs(navX.getYaw() - angStart) > diagonalBrokeThreshold) {
            handleCollision(distance);
            SetDrivePower(0);
            return true;
        }
        SetDrivePower(0);
        return false;
    }

    // Functions like the above method, but coasts instead of stopping. This is good for curved motion.
    public boolean DiagonalForwardsLeftCoast(double distance, double power) throws InterruptedException {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        double angStart = navX.getYaw();
        while (pastRange > distance && Math.abs(navX.getYaw() - angStart) < diagonalBrokeThreshold && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(power, -power, 0);
        }
        if (Math.abs(navX.getYaw() - angStart) > diagonalBrokeThreshold) {
            handleCollision(distance);
            SetDrivePower(0);
            return true;
        }
        return false;
    }

    public boolean DiagonalForwardsRight(double sensor, double power) throws InterruptedException {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        double angStart = navX.getYaw();
        while (pastRange > sensor && l.opModeIsActive() && (Math.abs(navX.getYaw() - angStart) < diagonalBrokeThreshold)) {
            pastRange = getRange(pastRange);
            arcadeMecanum(power, power, 0);
            return true;
        }
        if (Math.abs(navX.getYaw() - angStart) > diagonalBrokeThreshold) {
            handleCollision(sensor);
            return true;
        }

        SetDrivePower(0);
        return false;
    }

    public boolean DiagonalForwardsRight(double sensor, double yIn, double xIn) {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        while (pastRange > sensor && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(yIn, xIn, 0);
            SetDrivePower(0);
            return true;
        }
        SetDrivePower(0);
        return false;
    }

    public boolean DiagonalForwardsRightCoast(double sensor, double power) {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        while (pastRange > sensor && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(power, power, 0);
            return true;
        }
        return false;
    }

    public boolean DiagonalBackwardsRight(double sensor, double power) {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        while (pastRange > sensor && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(-power, power, 0);
            SetDrivePower(0);
            return true;
        }
        SetDrivePower(0);
        return false;
    }

    public boolean DiagonalBackwardsRight(double sensor, double yIn, double xIn) {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        while (pastRange > sensor && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(-yIn, xIn, 0);
            SetDrivePower(0);
            return true;
        }
        SetDrivePower(0);
        return false;
    }

    public void ToRangeByArcade(double sensor, double yIn, double xIn, double cIn) {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        while (pastRange > sensor && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(yIn, xIn, cIn);
        }
        SetDrivePower(0);
    }

    public boolean DiagonalBackwardsRightCoast(double sensor, double power) {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        while (pastRange > sensor && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(-power, power, 0);
            return true;
        }
        return false;
    }

    public boolean DiagonalBackwardsLeft(double sensor, double power) throws InterruptedException {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        double angStart = navX.getYaw();
        while (pastRange > sensor && Math.abs(navX.getYaw() - angStart) < diagonalBrokeThreshold && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(-power, -power, 0);
        }

        if (Math.abs(navX.getYaw() - angStart) > 25) {
            handleCollision(sensor);
            SetDrivePower(0);
            return true;
        }
        SetDrivePower(0);
        return false;
    }

    public boolean DiagonalBackwardsLeft(double sensor, double yIn, double xIn) throws InterruptedException {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();
        double angStart = navX.getYaw();
        while (pastRange > sensor && Math.abs(navX.getYaw() - angStart) < diagonalBrokeThreshold && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(-yIn, -xIn, 0);
        }
        if (Math.abs(navX.getYaw() - angStart) > diagonalBrokeThreshold) {
            handleCollision(sensor);
            return true;
        }
        SetDrivePower(0);
        return false;
    }

    public boolean DiagonalBackwardsLeftCoast(double sensor, double power) throws InterruptedException {
        double pastRange = 254;
        if (!l.opModeIsActive())
            Finish();

        double angStart = navX.getYaw();
        while (pastRange > sensor && Math.abs(navX.getYaw() - angStart) < diagonalBrokeThreshold && l.opModeIsActive()) {
            pastRange = getRange(pastRange);
            arcadeMecanum(-power, -power, 0);
        }
        if (Math.abs(navX.getYaw() - angStart) > diagonalBrokeThreshold) {
            handleCollision(sensor);
            return true;
        }
        return false;
    }

    public void ArcadeToAngleLeft(double yIn, double xIn, double cIn, double angleTarget) throws InterruptedException {
        arcadeMecanum(yIn, xIn, cIn);
        while (navX.getYaw() >= -Math.abs(angleTarget) && l.opModeIsActive()) {
            l.idle();
        }
        SetDrivePower(0);
    }

    public void ArcadeToAngleRight(double yIn, double xIn, double cIn, double angleTarget) throws InterruptedException {
        arcadeMecanum(yIn, xIn, cIn);
        while (navX.getYaw() <= Math.abs(angleTarget) && l.opModeIsActive()) {
            l.idle();
        }
        SetDrivePower(0);
    }

    public void ArcadeToAngleLeftCoast(double yIn, double xIn, double cIn, double angleTarget) throws InterruptedException {
        arcadeMecanum(yIn, xIn, cIn);
        while (navX.getYaw() >= -Math.abs(angleTarget) && l.opModeIsActive()) {
            l.idle();
        }
    }

    public void ArcadeToAngleRightCoast(double yIn, double xIn, double cIn, double angleTarget) throws InterruptedException {
        arcadeMecanum(yIn, xIn, cIn);
        while (navX.getYaw() <= Math.abs(angleTarget) && l.opModeIsActive()) {
            l.idle();
        }
    }


    // This is the same method as we use in TeleOp.
    // We use it for our diagonal strafing methods.
    public void arcadeMecanum(double y, double x, double c) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between -1 and +1, if not there already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        leftFrontWheel.setPower(leftFrontVal);
        leftBackWheel.setPower(leftBackVal);
        rightFrontWheel.setPower(rightFrontVal);
        rightBackWheel.setPower(rightBackVal);
    }

}
