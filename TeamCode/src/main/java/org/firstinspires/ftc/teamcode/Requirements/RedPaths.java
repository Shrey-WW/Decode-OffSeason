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

    public Pose startPose, GPPpose, PGPpose, PPGpose, scorePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2;

    public PathChain Intake2_, Intake1_, ShootPreloads, fillerPath, fillerPath2, move;

    public RedPaths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if (autoType == AutoType.FAR_NINE){
            buildFarNine();
        }
        else if (autoType == AutoType.CLOSE_NINE) {
            buildCloseNine();
        }
        else if(autoType == AutoType.CLOSE_TWELVE) {
            buildCloseTwelveNoTurret();
        }
        else if(autoType == AutoType.FAR_TWELVE) {
            buildFarTwelve();
        }
    }


    public void buildFarNine(){
        startPose = new Pose(56, 8.75).mirror();
        scorePose = new Pose(56,15).mirror();
        GPPpose = new Pose(8.25,35).mirror();
        PGPpose = new Pose(8.25,60).mirror();

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(65))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(43, 35).mirror()
                        )).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                .build();


        Intake1_ =  follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43, 35).mirror(), GPPpose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(GPPpose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(56, 60).mirror()))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.constant(Math.toRadians(65))),
                        new HeadingInterpolator.PiecewiseNode(.6,1, HeadingInterpolator.linear(Math.toRadians(65), Math.toRadians(0))
                        ))).build();

        Intake2_ = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(56.000, 60.000).mirror(), PGPpose)
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierLine(PGPpose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                .build();
        move = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(30,25).mirror()))
                .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
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

        fillerPath = follower.pathBuilder().addPath(
                new BezierLine(new Pose(8.75, 60).mirror(), new Pose(40, 60).mirror())
        ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(40, 60).mirror(), scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42)).build();

        fillerPath2 = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(50, 80).mirror()))
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0)).build();
    }
    public void buildFarTwelve(){
        startPose = new Pose(56, 8.75);
        scorePose = new Pose(56,15);
        PPGpose = new Pose(12, 35);
        PGPpose = new Pose(12 ,60);

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(56.2, 35),
                                PPGpose)
                ).setTangentHeadingInterpolation()
                .build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(PPGpose, scorePose)
        ).setTangentHeadingInterpolation().setReversed().build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(56, 59),
                                PGPpose)
                ).setTangentHeadingInterpolation()
                .build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierLine(PGPpose, scorePose)
                ).setTangentHeadingInterpolation().setReversed()
                .build();


    }
    public void buildCloseTwelveNoTurret(){
        startPose = new Pose(19,123);
        GPPpose = new Pose(18.5, 83);
        PGPpose = new Pose(13.5, 56);
        PPGpose = new Pose(13.5, 35);


        ShootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, new Pose(57, 85))
        ).setTangentHeadingInterpolation().setReversed().build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(57, 85), GPPpose)
        ).setTangentHeadingInterpolation().build();


    }

}
