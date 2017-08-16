package org.firstinspires.ftc.teamcode.VuforiaSamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Ethan Schaffer on 7/2/2017.
 */

@Autonomous(name = "Vuforia - Gear", group = "Autonomous")
public class VuforiaGearTracker extends LinearOpMode{
    public enum LeftOrRight {
        LEFT, RIGHT, CENTER
    }

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaGetter v = new VuforiaGetter();
        VuforiaTrackables beacons = v.getVuforia(1);

        //General Constants
        double MIN_POWER = .2;


        // Terms that change each loop
        double defaultPower;
        double powerL;
        double powerR;

        double distanceForwardsBackwards;
        double distanceLeftRight;
        double distanceUpDown;

        LeftOrRight sideOfPattern;
        boolean isGoingBackwards = false;

        waitForStart();

        beacons.activate();

        while(opModeIsActive()){
            VuforiaGetter.Pattern pattern = VuforiaGetter.Pattern.Gears;
            VuforiaGetter.DistanceOffsets distanceOffsets = v.getOffset(beacons, pattern);
            if(distanceOffsets.foundValues){
                distanceForwardsBackwards = distanceOffsets.distance;
                distanceLeftRight = distanceOffsets.horizontal;
                distanceUpDown = distanceOffsets.vertical;
                telemetry.addData(pattern + " Forwards", "%.2f in", distanceForwardsBackwards/UniversalContants.millimetersPerInch);
                if(distanceLeftRight > 0){
                    sideOfPattern = LeftOrRight.LEFT;
                } else {
                    distanceLeftRight = -distanceLeftRight;
                    sideOfPattern = LeftOrRight.RIGHT;
                }
                telemetry.addData(pattern + (sideOfPattern == LeftOrRight.LEFT ? " Left" : " Right"),
                        "%.2f mm", distanceLeftRight);

                if(distanceForwardsBackwards < 50) {
                    defaultPower = -.5;
                    isGoingBackwards = true;
                } else{
                    defaultPower = distanceForwardsBackwards/(1200);
                    if(defaultPower < MIN_POWER){
                        defaultPower = MIN_POWER;
                    }
                    isGoingBackwards = false;
                }

                if(sideOfPattern == LeftOrRight.LEFT){
                    //Robot is on Left
                    powerL = defaultPower - (isGoingBackwards ? - UniversalContants.addToHigherSide : UniversalContants.addToHigherSide);
                    powerR = defaultPower + (isGoingBackwards ? - UniversalContants.subtractFromLowerSide : UniversalContants.subtractFromLowerSide);
                } else {
                    //Robot is on Right
                    powerL = defaultPower + (isGoingBackwards ? - UniversalContants.addToHigherSide : UniversalContants.addToHigherSide);
                    powerR = defaultPower - (isGoingBackwards ? - UniversalContants.subtractFromLowerSide : UniversalContants.subtractFromLowerSide);
                }

                telemetry.addData("Left", "%.2f", powerL);
                telemetry.addData("Right", "%.2f", powerR);

            } else {
                telemetry.addLine("Found No Objects!");
            }
            telemetry.update();
        }
        beacons.deactivate();
    }
}
