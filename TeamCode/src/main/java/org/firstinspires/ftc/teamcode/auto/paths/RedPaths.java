package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.constants.AutoType;

public class RedPaths extends Paths {

    public RedPaths(AutoType autoType, Follower follower) {
        super(autoType, follower);
    }

    public void buildPaths() { super.buildPaths(); }
    
    public void buildCloseFifteen() {
        startPose = new Pose(19, 120, 2.4065).mirror();
        spike1 = new Pose(18, 82.5).mirror();
        spike2 = new Pose(9, 57).mirror();

        shootPreloads = follower.pathBuilder().addPath(
                new BezierLine(
                        startPose,
                        new Pose(53, 78).mirror()
                )
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.6, 1, HeadingInterpolator.constant(Math.toRadians(0)))
        )).build();

        intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(53, 78).mirror(),
                        new Pose(50, 57).mirror(),
                        spike2
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                spike2,
                                new Pose(22, 57).mirror(),
                                new Pose(53, 78).mirror()
                        )).setTangentHeadingInterpolation().setReversed().build();

        intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(53, 78).mirror(),
                        new Pose(22, 53).mirror(),
                        new Pose(8.8, 58.5).mirror()
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .45, HeadingInterpolator.tangent),
                new HeadingInterpolator.PiecewiseNode(.45, 1, HeadingInterpolator.constant(Math.toRadians(23)))
        )).build();

        intakeSweep1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(8.8, 58.5).mirror(),
                        new Pose(12, 57).mirror(),
                        new Pose(8, 55).mirror()
                )).setConstantHeadingInterpolation(Math.toRadians(20)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(8, 55).mirror(),
                        new Pose(22, 51).mirror(),
                        new Pose(53, 78).mirror()
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.6, 1, HeadingInterpolator.constant(Math.toRadians(-15)))
        )).build();

        intake3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54, 82.5).mirror(),
                                spike1
                        )).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                spike1,
                                new Pose(48, 110).mirror()
                        )).setConstantHeadingInterpolation(Math.toRadians(-45)).build();
    }

    protected void buildElliotFar(){
        startPose = new Pose(56.000, 8.500, Math.toRadians(90)).mirror();
        spike1 = new Pose(15, 39).mirror();
        scorePose = new Pose(55, 17).mirror();

        shootPreloads = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0)).build();

        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(53, 37.5).mirror(),
                                spike1
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                spike1,
                                scorePose
                        )
                ).setTangentHeadingInterpolation().setReversed().setGlobalDeceleration(.25).build();

        intakeSweep1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(8, 60).mirror(),
                                new Pose(8, 27).mirror()
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .5, HeadingInterpolator.linear(Math.toRadians(30), Math.toRadians(-90))),
                        new HeadingInterpolator.PiecewiseNode(.5, 1, HeadingInterpolator.constant(Math.toRadians(-90)))
                )).setNoDeceleration().build();

        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8, 27).mirror(),
                                new Pose(8, 9).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        new BezierCurve(
                                new Pose(8, 9).mirror(),
                                new Pose(8, 15).mirror(),
                                scorePose
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(0)))
                )).build();

        intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(16, 26).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(16, 26).mirror(),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        intake4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(16, 26).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(16, 26).mirror(),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(43, 17).mirror())
                ).setConstantHeadingInterpolation(Math.toRadians(0)).build();
    }
}