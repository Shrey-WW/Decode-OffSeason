package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.constants.AutoType;

public class BluePaths extends Paths {

    public BluePaths(AutoType autoType, Follower follower) {
        super(autoType, follower);
    }

    public void buildPaths() {
        super.buildPaths();
    }

    public void buildCloseFifteen() {
        startPose = new Pose(19, 121, 2.4065);
        spike1 = new Pose(18, 82.5);
        spike2 = new Pose(12, 57);

        shootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(53, 78)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.6, 1, HeadingInterpolator.constant(Math.toRadians(180)))
        )).build();

        intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(53, 78),
                        new Pose(50, 57),
                        spike2
                )).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(
                                spike2,
                                new Pose(22, 57),
                                new Pose(53, 78)
                        )).setTangentHeadingInterpolation().setReversed().build();

        intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(53, 78),
                        new Pose(22, 53),
                        new Pose(8.8, 60)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .45, HeadingInterpolator.tangent),
                new HeadingInterpolator.PiecewiseNode(.45, 1, HeadingInterpolator.constant(Math.toRadians(160)))
                )).build();

        intakeSweep1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(8.8, 60),
                        new Pose(10, 57),
                        new Pose(7, 57)
                )).setConstantHeadingInterpolation(Math.toRadians(163)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(7, 57),
                        new Pose(22, 51),
                        new Pose(54, 81.5)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.6, 1, HeadingInterpolator.constant(Math.toRadians(195)))
        )).build();

        intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(54, 81.5),
                        spike1
                )).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                spike1,
                                new Pose(52, 115)
                        )).setConstantHeadingInterpolation(Math.toRadians(225)).build();
    }

    protected void buildElliotFar(){
        startPose = new Pose(56.000, 8.500, Math.toRadians(90));
        spike1 = new Pose(15, 39);
        scorePose = new Pose(55, 17);

        shootPreloads = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();

        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(53, 37.5),
                                spike1
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                                new Pose(8, 60),
                                new Pose(8, 27)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .5, HeadingInterpolator.linear(Math.toRadians(150), Math.toRadians(270))),
                        new HeadingInterpolator.PiecewiseNode(.5, 1, HeadingInterpolator.constant(Math.toRadians(270)))
                )).setNoDeceleration().build();


        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8, 27),
                                new Pose(8, 9)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(
                        new BezierCurve(
                                new Pose(8, 9),
                                new Pose(8, 15),
                                scorePose
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(180)))
                )).build();

        intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(16, 26)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(16, 26),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        intake4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(16, 26)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(16, 26),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(43, 17))
                ).setConstantHeadingInterpolation(Math.toRadians(180)).build();
    }
}