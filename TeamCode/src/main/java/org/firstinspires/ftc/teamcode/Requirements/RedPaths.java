package org.firstinspires.ftc.teamcode.Requirements;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class RedPaths {

    private final AutoType autoType;

    private final Follower follower;

    public Pose startPose, GPPpose, PGPpose, PPGpose, scorePose, gatePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2, Intake3, goToScore3, openGate;

    public PathChain Intake2_, Intake1_, ShootPreloads, fillerPath, fillerPath2, move;

    public RedPaths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if (autoType == AutoType.CLOSE_TWELVE_NO_TURRET) {
            buildCloseTwelveNoTurret();
        }
    }


    public void buildCloseTwelveNoTurret(){
        startPose = new Pose(20,123).mirror(144);
        GPPpose = new Pose(21, 85.5).mirror(144);
        gatePose = new Pose(18, 74).mirror(144);
        PGPpose = new Pose(15, 56).mirror(144);
        PPGpose = new Pose(15, 39).mirror(144);

        ShootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, new Pose(55, 84.5).mirror(144))
        ).setConstantHeadingInterpolation(Math.toRadians(47)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(55, 84.5).mirror(144), GPPpose)
        ).setTangentHeadingInterpolation().build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        GPPpose,
                        new Pose(35, 80).mirror(144),
                        gatePose)
        ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(
                        GPPpose,
                        new Pose(54, 85.5).mirror(144))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(46))
                ))).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(54, 85.5).mirror(144),
                        new Pose(60, 57).mirror(144),
                        PGPpose)
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .5, HeadingInterpolator.linear(Math.toRadians(46), Math.toRadians(0))),
                new HeadingInterpolator.PiecewiseNode(.5, 1, HeadingInterpolator.constant(Math.toRadians(0))
                ))).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        PGPpose,
                        new Pose(56, 85.5).mirror(144))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(46))
                ))).build();

        Intake2_ = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(56, 85.5).mirror(144),
                        new Pose(56, 39).mirror(144))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.constant(Math.toRadians(90))),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(0))
                ))).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(56, 39).mirror(144),
                        PPGpose)
        ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore3 = follower.pathBuilder().addPath(
                new BezierLine(
                        PPGpose,
                        new Pose(56, 85.5).mirror(144)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(50))
                ))).build();

        move = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(56, 85.5).mirror(144),
                        new Pose(45, 81).mirror(144)
                )).setTangentHeadingInterpolation().build();


    }

}
