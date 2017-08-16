package org.firstinspires.ftc.teamcode.TankSamples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ethan Schaffer on 7/2/2017.
 */

public class SixWheelDrive {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    BNO055IMU imu;

    SixWheelDrive(LinearOpMode l, DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb) {
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        linearOpMode = l;
        leftFront = lf;
        leftBack = lb;
        rightFront = rf;
        rightBack = rb;
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void turnExact(double angle) {
        turnExact(.25, angle);
    }

    public void turnExact(double power, double angle) {
        turnExact(power, angle, 1.5);
    }

    public void turnExact(double power, double angle, double threshold) {
        while (opModeIsActive() && Math.abs(getHeading() - angle) > threshold) {
            if (getHeading() > angle) {
                setPowers(power, -power);
            } else {
                setPowers(-power, power);
            }
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
        setPowers(0);
    }

    public void turnRelative(double power, double angle) {
        turnRelative(power, angle, 1.5);
    }

    public void turnRelative(double power, double angle, double threshold) {
        turnExact(power, getHeading() + angle, threshold);
    }

    public void alignToWithinOf(double power, double angle, double threshold) {
        turnExact(power, angle + threshold);
        turnExact(power, angle - threshold);
        turnExact(power, angle + threshold);
    }

    public void moveTo(int distance, double power) {
        // Distance and Power should be the same sign!
        // If distance is greater than 0 (fowards)
        // and power is less than 0 (backwards),
        // the loop will never end. The same is true if
        // distance is less than 0 and power is greater than 0.
        if (distance / Math.abs(distance) != power / Math.abs(power)) {
            return;
        }
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setRelativeTargetPos(distance);
        setPowers(power);
        while (aMotorIsBusy()) {
            linearOpMode.idle();
        }
        setPowers(0);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveByEncoderPosition(int power, int position) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (leftFront.getCurrentPosition() < position) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
            rightFront.setPower(power);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void turnExactCoast(double angle) {
        turnExactCoast(.25, angle);
    }

    public void turnExactCoast(double power, double angle) {
        turnExactCoast(power, angle, 1.5);
    }

    public void turnExactCoast(double power, double angle, double threshold) {
        while (opModeIsActive() && Math.abs(getHeading() - angle) > threshold) {
            if (getHeading() > angle) {
                setPowers(power, -power);
            } else {
                setPowers(-power, power);
            }
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
    }

    public void turnRelativeCoast(double power, double angle) {
        turnRelativeCoast(power, angle, 1.5);
    }

    public void turnRelativeCoast(double power, double angle, double threshold) {
        turnExactCoast(power, getHeading() + angle, threshold);
    }

    public void moveToCoast(int distance, double power) {
        // Distance and Power should be the same sign!
        // If distance is greater than 0 (fowards)
        // and power is less than 0 (backwards),
        // the loop will never end. The same is true if
        // distance is less than 0 and power is greater than 0.
        if (distance / Math.abs(distance) != power / Math.abs(power)) {
            return;
        }
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setRelativeTargetPos(distance);
        setPowers(power);
        while (aMotorIsBusy()) {
            linearOpMode.idle();
        }
    }

    public void moveByEncoderPositionCoast(int power, int position) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (leftFront.getCurrentPosition() < position) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
            rightFront.setPower(power);
        }
    }

    public void setLeft(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
    }

    public void setRight(double power) {
        rightBack.setPower(power);
        rightFront.setPower(power);
    }

    public void setLeft(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
    }

    public void setRight(DcMotor.RunMode mode) {
        rightBack.setMode(mode);
        rightFront.setMode(mode);
    }

    public void setModes(DcMotor.RunMode mode) {
        setLeft(mode);
        setRight(mode);
    }

    public void resetDriveEncoders() {
        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while (leftFront.getCurrentPosition() != 0 || leftBack.getCurrentPosition() != 0
                || rightBack.getCurrentPosition() != 0 || rightFront.getCurrentPosition() != 0) {

        }
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPowers(double power) {
        setPowers(power, power);
    }

    public void setPowers(double powerL, double powerR) {
        setLeft(powerL);
        setRight(powerR);
    }

    public void setRelativeTargetPos(DcMotor m, int targetPos) {
        if (m.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        m.setTargetPosition(m.getCurrentPosition() + targetPos);
    }

    public void setRelativeTargetPos(int targetPos) {
        setRelativeTargetPos(leftFront, targetPos);
        setRelativeTargetPos(leftBack, targetPos);
        setRelativeTargetPos(rightBack, targetPos);
        setRelativeTargetPos(rightFront, targetPos);
    }

    public boolean aMotorIsBusy() {
        return leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy();
    }

    public enum LeftOrRight {
        LEFT, RIGHT, CENTER
    }

    public void driveToTargetStraight(VuforiaGetter.Pattern pattern, VuforiaTrackables patterns, VuforiaGetter vuforiaGetter) {
        double powerL;
        double powerR;
        double distanceForwardsBackwards;
        double distanceLeftRight;
        double distanceUpDown;
        double MIN_POWER = .25;
        LeftOrRight sideOfPattern;
        do {
            VuforiaGetter.DistanceOffsets distanceOffsets = vuforiaGetter.getOffset(patterns, pattern);
            if (distanceOffsets.foundValues) {
                distanceForwardsBackwards = distanceOffsets.distance;
                distanceLeftRight = distanceOffsets.horizontal;
                distanceUpDown = distanceOffsets.vertical;
                telemetry.addData(pattern + " Forwards", "%.2f in", distanceForwardsBackwards / UniversalContants.millimetersPerInch);
                if (distanceLeftRight > 0) {
                    sideOfPattern = LeftOrRight.LEFT;
                } else {
                    distanceLeftRight = -distanceLeftRight;
                    sideOfPattern = LeftOrRight.RIGHT;
                }
                telemetry.addData(pattern + (sideOfPattern == LeftOrRight.LEFT ? " Left" : " Right"),
                        "%.2f mm", distanceLeftRight);

                //BACKWARDS
                if (sideOfPattern == LeftOrRight.LEFT) {
                    powerL = (MIN_POWER - UniversalContants.addToHigherSide);
                    powerR = (MIN_POWER + UniversalContants.subtractFromLowerSide);
                } else {
                    powerL = (MIN_POWER + UniversalContants.subtractFromLowerSide);
                    powerR = (MIN_POWER - UniversalContants.addToHigherSide);
                }
                setLeft(powerL);
                setRight(powerR);
                telemetry.addData("Left", "%.2f", powerL);
                telemetry.addData("Right", "%.2f", powerR);
            } else {
                //Found No Values
                setPowers(0);
                telemetry.addLine("Found No Objects!");
                setPowers(-MIN_POWER, -MIN_POWER);
                //Back Up Until we see the target
                distanceForwardsBackwards = 101;
            }
            telemetry.update();
        }
        while (opModeIsActive() && (distanceForwardsBackwards / UniversalContants.millimetersPerInch > 8));
    }

    boolean opModeIsActive() {
        return this.linearOpMode.opModeIsActive();
    }
}
