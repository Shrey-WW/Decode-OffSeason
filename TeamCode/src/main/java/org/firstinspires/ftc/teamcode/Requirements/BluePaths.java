package org.firstinspires.ftc.teamcode.Requirements;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class BluePaths {

    private final AutoType autoType;

    private final Follower follower;

    public Pose startPose, endPose, GPPpose, PGPpose, PPGpose, scorePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2;

    public PathChain Intake2_, Intake1_, ShootPreloads;

    public PathChain turn1, turn2, turn3;

    public enum AutoType {
        FAR_TWELVE, CLOSE_TWELVE, CLOSE_NINE, FAR_NINE, NINE_TEST
    }

    public BluePaths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if (autoType == AutoType.FAR_NINE){
            buildLinearNine();
        }
        else if(autoType == AutoType.CLOSE_NINE) {
            buildCloseNine();
        }
        else if (autoType == AutoType.NINE_TEST){
            buildTestNine();
        }
    }

    private void buildLinearNine(){
        startPose = new Pose(56, 8.75,Math.PI/2);
        scorePose = new Pose(56,15, Math.toRadians(97));
        PPGpose = new Pose(8.25,35, Math.PI);
        PGPpose = new Pose(8.25,60, Math.PI);

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(97))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(
                        scorePose,
                        new Pose(43, 35)
                )).setLinearHeadingInterpolation(Math.toRadians(97), Math.toRadians(180))
                .build();


        Intake1_ =  follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43, 35), PPGpose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(PPGpose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, new Pose(56, 60)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.constant(Math.toRadians(110))),
                        new HeadingInterpolator.PiecewiseNode(.6,1, HeadingInterpolator.linear(Math.toRadians(110), Math.toRadians(180))
                ))).build();

        Intake2_ = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(56.000, 60.000), PGPpose)
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierLine(PGPpose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();


        /// turn to 97 degrees
    }


    public void buildCloseNine(){
        startPose = new Pose(21.500, 121.000, Math.PI/2);
        scorePose = new Pose(59,84, Math.toRadians(130));
        GPPpose = new Pose(18,84, Math.PI);
        PGPpose = new Pose(8.75,60, Math.PI);

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(startPose, scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130)).build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, GPPpose)
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .5, HeadingInterpolator.linear(Math.toRadians(130), Math.toRadians(180))),
                new HeadingInterpolator.PiecewiseNode(.5,1, HeadingInterpolator.constant(Math.toRadians(180))))
                ).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(18, 84), scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, new Pose(50, 60))
        ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180)).build();


        Intake2_ = follower.pathBuilder().addPath(
                new BezierLine(new Pose(50, 60), PGPpose)
        ).setConstantHeadingInterpolation(180).build();

        turn1 = follower.pathBuilder().addPath(
                new BezierLine(PGPpose, new Pose(50, 60))
        ).setConstantHeadingInterpolation(180).build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50, 60),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .setReversed()
                .build();

        /// turn to 130 degrees


    }

    public void buildTestNine(){
        startPose = new Pose(56, 8.75,Math.PI/2);
        scorePose = new Pose(56,15, Math.toRadians(113));
        PPGpose = new Pose(8.25,35, Math.PI);
        PGPpose = new Pose(8.25,60, Math.PI);

    }

}
