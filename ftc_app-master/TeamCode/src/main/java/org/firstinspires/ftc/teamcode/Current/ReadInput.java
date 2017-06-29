package org.firstinspires.ftc.teamcode.Current;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Input", name = "Input")
public class ReadInput extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(ReadInput.this, hardwareMap, telemetry, true);
        String FILE_DIR = "/Download";
        File file = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + FILE_DIR, "route.txt");
        Scanner s = null;
        while (!isStarted()) {
            robot.Housekeeping();
        }
        waitForStart();
        double startTime = getRuntime();

        try {
            s = new Scanner(file);
            while (s.hasNextDouble()) {
                double functionVal;
                functionVal = s.nextDouble();
                switch ((int)functionVal){
                    case 0:
                        robot.Finish();
                        break;
                    case 1:
                        robot.Move(s.nextDouble(), s.nextDouble());
                        break;
                    case 2:
                        //Degrees, Power
                        robot.TurnLeft(s.nextDouble(), s.nextDouble());
                        break;
                    case 3:
                        //Degrees, Power
                        robot.TurnRight(s.nextDouble(), s.nextDouble());
                        break;
                    case 4:
                        //Distance, Power
                        robot.StrafeToWall(s.nextDouble(), s.nextDouble());
                        break;
                    case 5:
                        //Distance, Power
                        robot.StrafeFromWall(s.nextDouble(), s.nextDouble());
                        break;
                    case 6:
                        robot.AlignToWithin(1.5, .05);
                        break;
                    case 7:
                        robot.EnableShot(0, 1);
                        break;
                    case 8:
                        robot.FindAndPressSquareToBeacon(s.nextDouble() == 1 ? Robot.team.Blue : Robot.team.Red, s.nextDouble());
                        break;
                    case 9:
                        double timeTarg = s.nextDouble();
                        while(getRuntime()-startTime > timeTarg){
                            sleep(1);
                        }
                        break;
                    case 10:
                        double color = s.nextDouble();
                        robot.CheckBeacon(color == 1 ? Robot.team.Blue : Robot.team.Red);
                        break;
                    case 11:
                        robot.ShootByVoltage();
                        break;
                    case 12:
                        robot.StopShooter();
                        break;
                    case 13:
                        sleep((long)s.nextDouble());
                        break;
                    case 14:
                        robot.DiagonalForwardsLeft(s.nextDouble(), s.nextDouble());
                        break;
                    case 15:
                        robot.DiagonalForwardsRight(s.nextDouble(), s.nextDouble());
                        break;
                    case 16:
                        robot.DiagonalBackwardsLeft(s.nextDouble(), s.nextDouble());
                        break;
                    case 17:
                        robot.DiagonalBackwardsRight(s.nextDouble(), s.nextDouble());
                        break;
                    case 18:
                        // Angle, Power
                        robot.AlignToWithin(s.nextDouble(), s.nextDouble());
                        break;
                    case 19:
                        robot.AlignToWithinOf(s.nextDouble(), s.nextDouble(), s.nextDouble());
                        break;
                    case 20:
                        //None, Power of Infeed
                        robot.EnableShot(0, s.nextDouble());
                        break;
                    case 21:
                        // Threshold, Power
                        robot.MoveByDelta(s.nextDouble(), s.nextDouble());
                        break;
                    case 22:
                        robot.ForwardsPLoop(s.nextDouble(), s.nextDouble());
                        break;
                    case 23:
                        robot.TurnLeftEnc(s.nextDouble(), s.nextDouble());
                        break;
                    case 24:
                        robot.TurnRightEnc(s.nextDouble(), s.nextDouble());
                        break;
                    case 25:
                        robot.StrafeToPrecise(s.nextDouble(), s.nextDouble(), s.nextDouble());
                        break;
                    case 26:
                        robot.TurnRightRelative(s.nextDouble(), s.nextDouble());
                        break;
                    case 27:
                        robot.TurnLeftRelative(s.nextDouble(), s.nextDouble());
                        break;
                    case 28:
                        robot.TurnLeftAbsolute(s.nextDouble(), s.nextDouble());
                        break;
                    case 29:
                        robot.TurnRightAbsolute(s.nextDouble(), s.nextDouble());
                        break;
                    case 30:
                        robot.arcadeMecanum(s.nextDouble(), s.nextDouble(), s.nextDouble());
                        break;
                    case 31:
                        robot.infeed.setPower(s.nextDouble());
                        break;
                    case 32:
                        double power = s.nextDouble();
                        robot.shoot1.setPower(power);
                        robot.shoot2.setPower(power);
                        break;
                    case 33:
                        robot.ArcadeToAngleLeft(s.nextDouble(), s.nextDouble(), s.nextDouble(), s.nextDouble());
                        break;
                    case 34:
                        robot.ArcadeToAngleRight(s.nextDouble(), s.nextDouble(), s.nextDouble(), s.nextDouble());
                    default:
                        robot.Finish();
                        break;
                }
            }
        } catch (FileNotFoundException e){
            e.printStackTrace();
        }

    }
}