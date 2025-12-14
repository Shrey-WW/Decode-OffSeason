package org.firstinspires.ftc.teamcode.Requirements;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class RedPaths {

    private final RedPaths.AutoType autoType;

    private final Follower follower;

    public Pose startPose, endPose, GPPpose, PGPpose, PPGpose, scorePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2;

    public PathChain Intake2_, Intake1_, ShootPreloads,turn1, turn2, move;

    public enum AutoType {
        FAR_TWELVE, CLOSE_TWELVE, CLOSE_NINE, FAR_NINE
    }

    public RedPaths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if (autoType == AutoType.FAR_NINE){
            buildLinearNine();
        }
        else if (autoType == AutoType.CLOSE_NINE){
            buildCloseNine();
        }
    }


    public void buildLinearNine(){
        startPose = new Pose(88, 8.75);
        scorePose = new Pose(88,10);
        GPPpose = new Pose(134,35);
        PGPpose = new Pose(134,60);

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(83))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(101, 35)
                        )).setLinearHeadingInterpolation(Math.toRadians(83), Math.toRadians(0))
                .build();

        Intake1_ =  follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(101, 35), GPPpose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(GPPpose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(88, 60)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.constant(Math.toRadians(70))),
                        new HeadingInterpolator.PiecewiseNode(.7,1, HeadingInterpolator.linear(Math.toRadians(70), Math.toRadians(0))
                        ))).build();

        Intake2_ = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(88, 60, Math.toRadians(0)), PGPpose)
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierLine(PGPpose, scorePose)
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                .build();
        move = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(56,30).mirror()))
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(180))
                .build();

    }

    public void buildCloseNine(){
        startPose = new Pose(19,123).mirror();
        scorePose = new Pose(50,93).mirror();
        PPGpose = new Pose(18,84).mirror();

        ShootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose,scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(42)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, new Pose(50, 84).mirror())
        ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0)).build();

        Intake1_ = follower.pathBuilder().addPath(
                new BezierLine(new Pose(50, 84).mirror(), PPGpose)
        ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(PPGpose, scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, new Pose(50, 58).mirror())
        ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0)).build();

        Intake2_ = follower.pathBuilder().addPath(
                new BezierLine(new Pose(50, 58).mirror(), new Pose(8.75, 60).mirror())
        ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        turn1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(8.75, 60).mirror(), new Pose(40, 60).mirror())
        ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(40, 60).mirror(), scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42)).build();

        turn2 = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(50, 80).mirror()))
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0)).build();
    }
}
