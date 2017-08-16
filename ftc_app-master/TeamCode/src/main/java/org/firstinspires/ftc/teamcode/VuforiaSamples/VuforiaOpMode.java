package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ethan Schaffer on 7/2/2017.
 */

@Autonomous(name = "Vuforia General", group = "Autonomous")
public class VuforiaOpMode extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaGetter v = new VuforiaGetter();
        VuforiaTrackables beacons = v.getVuforia();
        waitForStart();

        beacons.activate();
        while(opModeIsActive()){
            boolean foundOne = false;
            for(VuforiaTrackable beacon : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();
                if(pose != null) {
                    foundOne = true;
                    VectorF translation = pose.getTranslation();
                    double distanceForwardsBackwards = Math.abs(pose.getTranslation().get(2));
                    double distanceLeftRight = pose.getTranslation().get(0);
                    double distanceUpDown = pose.getTranslation().get(1);
                    // double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                    telemetry.addData(beacon.getName() + " Forwards", distanceForwardsBackwards);
                    telemetry.addData(beacon.getName() + (distanceLeftRight > 0 ? " Left" : " Right"), distanceLeftRight);
                }
            }
            if(!foundOne){
                telemetry.addData("Found No Objects", "Uh Oh!");
            }
            telemetry.update();
        }

    }
}
