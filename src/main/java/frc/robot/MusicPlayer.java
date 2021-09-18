package frc.robot;
import java.util.Collection;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.*;
public class MusicPlayer {
    public Orchestra orchestra;

    public MusicPlayer(){
        orchestra = new Orchestra();
    }

    enum Song{
        AMONG_US("/Users/zachl/Desktop/Spartabots/FRC-2020-master-test/src/main/java/frc/res/amongus.chrp"),
        CANDYLAND("/Users/zachl/Desktop/Spartabots/FRC-2020-master-test/src/main/java/frc/res/candyland.chrp");

        String path;
        Song(String path){
            this.path = path;
        }
    }

    public void add(TalonFX inst){
        orchestra.addInstrument(inst);
    }
    public void play(Song song){
        orchestra.loadMusic(song.path);
        orchestra.play();
    }

    public void pause(){
        if(orchestra.isPlaying())
            orchestra.pause();
    }

    public void resume(){
        orchestra.play();
    }
}
