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

    public PathChain Intake2_, Intake1_, ShootPreloads;

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
        startPose = new Pose(88, 8.75,Math.PI/2).mirror();
        scorePose = new Pose(88,15, Math.toRadians(97));
        GPPpose = new Pose(134,35, Math.PI).mirror();
        PGPpose = new Pose(134,60, Math.PI).mirror();

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

        /// turn to *70* degrees

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

        /// turn to *85* degrees
    }

    public void buildCloseNine(){
        startPose = new Pose(21.500, 121.000, Math.PI/2).mirror();
        scorePose = new Pose(59,84, Math.toRadians(130)).mirror();
        PPGpose = new Pose(18,84, Math.PI).mirror();
        PGPpose = new Pose(8.75,60, Math.PI).mirror();

        ShootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, new Pose(18, 84).mirror())
        ).setTangentHeadingInterpolation().build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(18, 84).mirror(), scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(18, 84).mirror(), new Pose(50, 59).mirror())
        ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0)).build();


        Intake2_ = follower.pathBuilder().addPath(
                new BezierLine(new Pose(18, 84).mirror(), new Pose(8.75, 59).mirror())
        ).setTangentHeadingInterpolation().build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.75, 59).mirror(),
                                new Pose(40, 59).mirror(),
                                scorePose
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /// turn to 50 degrees

    }
}
