package org.firstinspires.ftc.teamcode.NavigationVuforiaSamples;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class VuforiaGetter {
    public enum Pattern {
        Wheels, Tools, Lego, Gears
    }
    public int getPatternNumber(Pattern pattern){
        switch (pattern){
            case Wheels:
                return 0;
            case Tools:
                return 1;
            case Lego:
                return 2;
            default:
                return 3;
        }
    }

    public VuforiaTrackables getVuforia(){
        return getVuforia(true, 4);
    }
    public VuforiaTrackables getVuforia(int number) {
        return getVuforia(true, number);
    }
    public VuforiaTrackables getVuforia(boolean showCameraView, int number){
        if(number < 0){
            return getVuforia(Math.abs(number));
        }
        if(number > 10){
            return getVuforia();
        }
        VuforiaLocalizer.Parameters parameters;
        if(showCameraView) {
            parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
            // This makes the Vuforia view appear on the phone screen
            // Can remove the R.id.cameraMonitorViewId to save battery or whatever.
            // Not a huge need to
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }
        parameters.cameraDirection = UniversalContants.camera;
        // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        // Back is selfie camera. Use it when we are done testing.

        parameters.vuforiaLicenseKey = UniversalContants.vuforiaLicenceKey;
        // https://developer.vuforia.com/targetmanager/licenseManager/licenseListingDetails
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        // Can also use a teapot.
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        // Does what you think it would
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, number);
        // Vuforia can track all 4 vision targets. Change this is there are more or less targets.
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        return beacons;
    }
    public DistanceOffsets getOffset(VuforiaTrackables vuforiaTrackables, Pattern pattern){
        return getOffset(vuforiaTrackables, getPatternNumber(pattern));
    }

    public DistanceOffsets getOffset(VuforiaTrackables vuforiaTrackables, int value){
        VuforiaTrackable target = vuforiaTrackables.get(value);
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getPose();
        if(pose != null) {
            double distanceLeftRight = pose.getTranslation().get(0)-UniversalContants.robotHorizontalOffset;
            double distanceUpDown = pose.getTranslation().get(1)-UniversalContants.robotVerticalOffset;
            double distanceForwardsBackwards = Math.abs(pose.getTranslation().get(2))-UniversalContants.robotFrontOffset;
            return new DistanceOffsets(distanceForwardsBackwards, distanceLeftRight, distanceUpDown);
        }
        return new DistanceOffsets();
        //Found nothing value
    }

    public class DistanceOffsets{
        public double distance, horizontal, vertical;
        public boolean foundValues;
        DistanceOffsets(double forwardsBack, double leftRight, double upDown){
            distance = forwardsBack;
            horizontal = leftRight;
            vertical = upDown;
            foundValues = true;
        }
        DistanceOffsets(){
            this.foundValues = false;
            distance = horizontal = vertical = 0;
        }
    }
}
