package org.firstinspires.ftc.teamcode.NavigationVuforiaSamples;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class UniversalContants {
    public static final double robotHorizontalOffset = 100;
    public static final double robotVerticalOffset = 10;
    public static final double robotFrontOffset = 5;
    public static final double millimetersPerInch = 25.4;

    public static final double degreesPerRadian = 2*Math.PI/360;

    public static final double addToHigherSide = .02;
    public static final double subtractFromLowerSide = .01;

    public static final String leftFrontDrive = "lf";
    public static final String leftBackDrive = "lb";
    public static final String rightFrontDrive = "rf";
    public static final String rightBackDrive = "rb";
    public static final String vuforiaLicenceKey = "AfVNPjT/////AAAAGWxt1A0qnE/0ubDxBQVByN5Rb1GNo+3vvrqiIVpsnNHDWKyEcVhuKt6W/wPMw/0/wJh0iMnrWM+HddaZeSV8uGaUacthmkOT/xVt/+A+hlgt+3rIkDKkAYfIOw/DCK/RNY5U1LWCFSPGdjt5w3BQEg3iOEWzuhyovpBn+UlS56UhH6q5wP9qz9PpabM1Q7IW9MoYUGKTsiiLCQbB7ICHckbAQQBU1WJQdD3fAGnOOM0Dh1yjtOBlU3+kFZQOxKXNiVr5xxAZm903atvnT179VaATgl8U2yN9IW6h9gZ8tghlubFyeLRSiMaOrv/0gYchtBHH51WBJEsy2Tv2rzt403b4QMuZE5zQTRiZPJuJn83L";
    public static final VuforiaLocalizer.CameraDirection camera = VuforiaLocalizer.CameraDirection.FRONT;
    //Back is Selfie Camera

}
