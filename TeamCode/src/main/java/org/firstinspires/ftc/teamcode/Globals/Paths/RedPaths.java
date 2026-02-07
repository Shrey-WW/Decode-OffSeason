package org.firstinspires.ftc.teamcode.Globals.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Globals.Paradigms.Paths;
import org.firstinspires.ftc.teamcode.Globals.States.AutoType;

public class RedPaths extends Paths{

    public RedPaths(AutoType type, Follower f){
        super(type, f);
    }

    public void buildPaths(){
        if (autoType == AutoType.CLOSE_TWELVE_NO_TURRET) {
            buildCloseTwelveNoTurret();
        }
    }


    public void buildCloseTwelveNoTurret(){
        startPose = new Pose(19, 121).mirror(144);
        Spike1 = new Pose(19, 84).mirror(144);
        Spike2 = new Pose(21, 55).mirror(144);
        Spike3 = new Pose(21, 42).mirror(144);

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(55.500, 85.500).mirror(144)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 85.5).mirror(144),
                        new Pose(55.5, 59).mirror(144),
                        Spike2
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        Spike2,
                        new Pose(24.5, 61).mirror(144),
                        new Pose(20, 68).mirror(144)
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(18.5, 68).mirror(144),
                        new Pose(48,70).mirror(144),
                        new Pose(55.5, 85.5).mirror(144)
                )).setTangentHeadingInterpolation().setReversed().build();

        openGate2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 85.5).mirror(144),
                        new Pose(48, 70).mirror(144),
                        new Pose(20, 68).mirror(144)
                )).setTangentHeadingInterpolation().build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(20, 68).mirror(144),
                        new Pose(23, 62).mirror(144),
                        new Pose(15.5, 56).mirror(144)
                )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(16, 56).mirror(144),
                        new Pose(42, 67.5).mirror(144),
                        new Pose(58, 84).mirror(144)
                )).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(58, 84).mirror(144),
                        Spike1
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore3 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike1,
                        new Pose(38.5, 100).mirror(144)
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();


    }
    public void buildCloseTwelve() {
        startPose = new Pose(19, 121).mirror();
        Spike1 = new Pose(21, 84).mirror();
        Spike2 = new Pose(21, 55).mirror();
        Spike3 = new Pose(21, 42).mirror();

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(55.500, 85.500).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 85.5).mirror(),
                        new Pose(55.5, 59).mirror(),
                        Spike2
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        Spike2,
                        new Pose(24.5, 61).mirror(),
                        new Pose(20, 68).mirror()
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(18.5, 68).mirror(),
                        new Pose(48,70).mirror(),
                        new Pose(55.5, 85.5).mirror()
                )).setTangentHeadingInterpolation().setReversed().build();

        openGate2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 85.5).mirror(),
                        new Pose(48, 70).mirror(),
                        new Pose(20, 68).mirror()
                )).setTangentHeadingInterpolation().build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(20, 68).mirror(),
                        new Pose(23, 62).mirror(),
                        new Pose(15.5, 56).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(16, 56).mirror(),
                        new Pose(42, 67.5).mirror(),
                        new Pose(58, 84).mirror()
                )).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(58, 84).mirror(),
                        Spike1
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore3 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike1,
                        new Pose(38.5, 95).mirror()
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        leave = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(38.5, 95).mirror(),
                        new Pose(28, 95).mirror()
                )).setConstantHeadingInterpolation(Math.toRadians(0)).build();

    }
    public void buildFarTwelveNoTurret(){}
    public void buildExodus(){}


}
