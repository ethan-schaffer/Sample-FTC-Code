package org.firstinspires.ftc.teamcode.AudioSample;


import android.app.Activity;
import android.media.MediaPlayer;

import java.io.IOException;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

public class AudioHandler extends Activity {
    MediaPlayer mediaPlayer;
    String PATH_TO_FILE = "/sdcard/music.mp3";

    public AudioHandler() throws IOException {
        mediaPlayer = new  MediaPlayer();
        mediaPlayer.setDataSource(PATH_TO_FILE);
        mediaPlayer.prepare();
    }

    public boolean isPlaying() {
        return mediaPlayer.isPlaying();
    }

    public void playSound() {
        mediaPlayer.start();
    }

    public void setLooping(boolean b) {
        mediaPlayer.setLooping(b);
    }

    public void playSoundAsLoop() {
        setLooping(true);
        mediaPlayer.start();
    }

    public void pauseSound() {
        if (mediaPlayer.isPlaying()) {
            mediaPlayer.pause();
        }
    }

    public void stopSound() {
        if (mediaPlayer.isPlaying()) {
            mediaPlayer.stop();
        }
    }

}
