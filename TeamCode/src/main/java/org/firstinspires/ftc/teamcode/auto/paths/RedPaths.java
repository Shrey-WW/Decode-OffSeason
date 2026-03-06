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
        startPose = new Pose(19, 121, 2.4065).mirror(144);
        spike1 = new Pose(22, 83.5).mirror(144);
        spike2 = new Pose(12, 57).mirror(144);

        shootPreloads = follower.pathBuilder().addPath(
                new BezierLine(
                        startPose,
                        new Pose(53, 78).mirror(144)
                )
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.6, 1, HeadingInterpolator.constant(Math.toRadians(0)))
        )).build();

        intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(53, 78).mirror(144),
                        new Pose(50, 57).mirror(144),
                        spike2
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                spike2,
                                new Pose(22, 57).mirror(144),
                                new Pose(53, 78).mirror(144)
                        )).setTangentHeadingInterpolation().setReversed().build();

        intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(53, 78).mirror(144),
                        new Pose(22, 53).mirror(144),
                        new Pose(8.8, 60.7).mirror(144)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .45, HeadingInterpolator.tangent),
                new HeadingInterpolator.PiecewiseNode(.45, 1, HeadingInterpolator.constant(Math.toRadians(23)))
        )).build();

        intakeSweep1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(8.8, 60.7).mirror(144),
                        new Pose(11, 59).mirror(144),
                        new Pose(8.8, 58).mirror(144)
                )).setConstantHeadingInterpolation(Math.toRadians(20)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(8.8, 58).mirror(144),
                        new Pose(22, 51).mirror(144),
                        new Pose(53, 78).mirror(144)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.6, 1, HeadingInterpolator.constant(Math.toRadians(-15)))
        )).build();

        intake3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(53, 78).mirror(144),
                        new Pose(22, 53).mirror(144),
                        new Pose(8.8, 60.7).mirror(144)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .45, HeadingInterpolator.tangent),
                new HeadingInterpolator.PiecewiseNode(.45, 1, HeadingInterpolator.constant(Math.toRadians(23)))
        )).build();

        intakeSweep2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(8.8, 60.7).mirror(144),
                        new Pose(11, 59).mirror(144),
                        new Pose(8.8, 58).mirror(144)
                )).setConstantHeadingInterpolation(Math.toRadians(20)).build();

        goToScore3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(8.8, 58).mirror(144),
                        new Pose(30, 51).mirror(144),
                        new Pose(55, 83.5).mirror(144)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.6, 1, HeadingInterpolator.constant(Math.toRadians(0)))
        )).build();

        intake4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54, 83.5).mirror(144),
                                spike1
                        )).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                spike1,
                                new Pose(48, 110).mirror(144)
                        )).setConstantHeadingInterpolation(Math.toRadians(-45)).build();
    }

    public void buildElliotFar(){
        startPose = new Pose(56.000, 8.500, Math.toRadians(90)).mirror(144);
        spike1 = new Pose(15, 39).mirror(144);
        scorePose = new Pose(55, 17).mirror(144);

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
                                new Pose(53, 37.5).mirror(144),
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
                                new Pose(8, 60).mirror(144),
                                new Pose(8, 27).mirror(144)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .5, HeadingInterpolator.linear(Math.toRadians(30), Math.toRadians(-90))),
                        new HeadingInterpolator.PiecewiseNode(.5, 1, HeadingInterpolator.constant(Math.toRadians(-90)))
                )).setNoDeceleration().build();

        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8, 27).mirror(144),
                                new Pose(8, 9).mirror(144)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        new BezierCurve(
                                new Pose(8, 9).mirror(144),
                                new Pose(8, 15).mirror(144),
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
                                new Pose(16, 26).mirror(144)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(16, 26).mirror(144),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        intake4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(16, 26).mirror(144)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(16, 26).mirror(144),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(43, 17).mirror(144))
                ).setConstantHeadingInterpolation(Math.toRadians(0)).build();
    }
}