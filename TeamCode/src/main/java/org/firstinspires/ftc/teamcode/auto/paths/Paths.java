package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.constants.AutoType;

public abstract class Paths {
    protected final AutoType autoType;
    protected final Follower follower;

    protected Pose startPose, spike1, spike2, spike3, gatePose;

    public PathChain intake1, goToScore1, intake2, goToScore2, intake3, goToScore3, intake4, goToScore4;
    public PathChain preIntake2, preIntake3, shootPreloads, preIntake4;

    public PathChain intakeSweep1, intakeSweep2, leave, openGate, openGate2, openGate3;

    public Paths(AutoType autoType, Follower follower) {
        this.follower = follower;
        this.autoType = autoType;
    }

    public void buildPaths() {
        switch (autoType) {
            case CLOSE_TWELVE_NO_TURRET: buildCloseTwelveNoTurret(); break;
            case FAR_TWELVE_NO_TURRET:   buildFarTwelveNoTurret();   break;
            case CLOSE_TWELVE:           buildCloseTwelve();         break;
            case CLOSE_15:               buildCloseFifteen();        break;
            case ELLIOT_FAR:             buildElliotFar();           break;
        }
    }

    public abstract void buildCloseTwelveNoTurret();
    public abstract void buildFarTwelveNoTurret();
    public abstract void buildCloseTwelve();
    public abstract void buildCloseFifteen();
    public abstract void buildElliotFar();
}