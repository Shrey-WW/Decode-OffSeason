package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class Paths {

    private final AutoType autoType;

    private final Follower follower;

    public Pose startPose, endPose, GPPpose, PGPpose, PPGpose, scorePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2;

    public PathChain Intake2_;

    public enum AutoType {
        FAR_TWELVE, CLOSE_TWELVE, FAR_NINE, CLOSE_NINE, LINEAR_NINE
    }

    public Paths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if (autoType == AutoType.FAR_NINE){
            buildFarNine();
        }
        else if (autoType == AutoType.LINEAR_NINE){
            buildLinearNine();
        }
    }

    private void buildFarNine(){
        startPose = new Pose(56, 8.75);
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

    private void buildLinearNine(){
        startPose = new Pose(56, 8.75,Math.PI/2);
        scorePose = new Pose(56,15, Math.PI/2);
        PPGpose = new Pose(16,37.5, Math.PI);
        PGPpose = new Pose(16,60, Math.PI);
        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        startPose,
                        new Pose(56.000, 37.5),
                        PPGpose
                )).setTangentHeadingInterpolation()
                .build();
        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(PPGpose, scorePose))
                .setTangentHeadingInterpolation().setReversed()
                .build();
        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, new Pose(56, 60, Math.toRadians(180))))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.constant(Math.toRadians(90))),
                        new HeadingInterpolator.PiecewiseNode(.7,1, HeadingInterpolator.linear(Math.toRadians(90), Math.toRadians(180))
                ))).build();
        Intake2_ = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(56.000, 60.000, Math.toRadians(180)), PGPpose)
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        goToScore2 = follower.pathBuilder().addPath(
                new BezierLine(PGPpose, scorePose)
                ).setTangentHeadingInterpolation().setReversed()
                .build();
    }

}
