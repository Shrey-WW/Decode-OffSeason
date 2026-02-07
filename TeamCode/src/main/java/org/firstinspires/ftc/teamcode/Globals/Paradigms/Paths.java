package org.firstinspires.ftc.teamcode.Globals.Paradigms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Globals.States.AutoType;

public abstract class Paths {
    protected final AutoType autoType;

    protected final Follower follower;

    protected Pose startPose, Spike1, Spike2, Spike3, scorePose, gatePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2, Intake3, goToScore3, Intake4, goToScore4;
    public PathChain Intake2_, Intake1_, ShootPreloads, Intake4_;

    public  PathChain fillerPath, fillerPath2, leave, openGate, openGate2;


    public Paths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if(autoType == AutoType.CLOSE_TWELVE_NO_TURRET) {
            buildCloseTwelveNoTurret();
        }
        else if(autoType == AutoType.FAR_TWELVE_NO_TURRET) {
            buildFarTwelveNoTurret();
        }
        else if(autoType == AutoType.EXODUS){
            buildExodus();
        }
        else if (autoType == AutoType.CLOSE_TWELVE){
            buildCloseTwelve();
        }
    }

    public abstract void buildCloseTwelveNoTurret();
    public abstract void buildFarTwelveNoTurret();
    public abstract void buildCloseTwelve();
    public abstract void buildExodus();
}
