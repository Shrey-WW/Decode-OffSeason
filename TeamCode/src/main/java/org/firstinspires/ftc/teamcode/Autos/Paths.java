package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    private final AutoType autoType;
    private Follower follower;

    public Pose startPose, endPose, GPPpose, PGPpose, PPGpose, scorePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2;

    public enum AutoType {
        FAR_TWELVE, CLOSE_TWELVE, FAR_NINE, CLOSE_NINE
    }

    public Paths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if (autoType == AutoType.FAR_NINE){
            buildFarNine();
        }
    }

    private void buildFarNine(){
        startPose = new Pose(56, 8);
        endPose = new Pose(56,14);
        PPGpose = new Pose(16,37);
        PGPpose = new Pose(16,60);
        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(56.000, 39.000),
                                PPGpose
                        )).setTangentHeadingInterpolation().build();

        goToScore1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PPGpose,
                                new Pose(56, 37),
                                endPose
                        )).setTangentHeadingInterpolation().build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                endPose,
                                new Pose(56.000, 63),
                                PGPpose
                        )).setTangentHeadingInterpolation().build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                PGPpose,
                                new Pose(56, 60),
                                endPose
                        )).setTangentHeadingInterpolation().build();

    }

}
