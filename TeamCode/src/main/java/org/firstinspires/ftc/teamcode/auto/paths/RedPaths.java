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

    public void buildCloseTwelveNoTurret() {
        startPose = new Pose(19, 121).mirror(144);
        spike1 = new Pose(24, 85.5).mirror(144);
        gatePose = new Pose(20, 72).mirror(144);
        spike2 = new Pose(22, 58).mirror(144);
        spike3 = new Pose(22, 40).mirror(144);

        shootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, new Pose(55, 85.5).mirror(144))
        ).setConstantHeadingInterpolation(Math.toRadians(50)).build();

        intake1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(55, 85.5).mirror(144), spike1)
        ).setTangentHeadingInterpolation().build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        spike1,
                        new Pose(35, 80).mirror(144),
                        gatePose)
        ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(gatePose, new Pose(55, 85.5).mirror(144))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(49))
                ))).build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55, 85.5).mirror(144), new Pose(55, 59).mirror(144)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Pose(55, 59).mirror(144), spike2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(spike2, new Pose(55, 85.5).mirror(144)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(52))
                        ))).build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55, 85.5).mirror(144), new Pose(57, 40).mirror(144)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(0))
                        )))
                .addPath(new BezierLine(new Pose(57, 40).mirror(144), spike3))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(spike3, new Pose(55, 115).mirror(144)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(15))
                        ))).build();
    }
    public void buildFarTwelveNoTurret() {
    }

    public void buildCloseFifteen() {
    }

    public void buildElliotFar(){
    }
}