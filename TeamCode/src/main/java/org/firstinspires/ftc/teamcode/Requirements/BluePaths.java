package org.firstinspires.ftc.teamcode.Requirements;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class BluePaths {

    private final AutoType autoType;

    private final Follower follower;

    public Pose startPose, endPose, GPPpose, PGPpose, PPGpose, scorePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2;

    public PathChain Intake2_, Intake1_, ShootPreloads;

    public PathChain turn1, turn2, move, openGate, goToScore3;

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

        else if(autoType == AutoType.FAR_TWELVE) {
            buildFarTwelve();
        }
    }

    private void buildLinearNine(){
        startPose = new Pose(56, 8.75);
        scorePose = new Pose(56,15);
        PPGpose = new Pose(8.25,35);
        PGPpose = new Pose(8.25,60);

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(115))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(
                        scorePose,
                        new Pose(43, 35)
                )).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180))
                .build();


        Intake1_ =  follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43, 35), PPGpose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(PPGpose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
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
        move = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(30,25)))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(0))
                .build();

    }


    public void buildCloseNine(){
        startPose = new Pose(19,123);
        scorePose = new Pose(50,93);
        GPPpose = new Pose(18,84);

        ShootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(143)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, new Pose(50, 84))
        ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180)).build();


        Intake1_ = follower.pathBuilder().addPath(
                new BezierLine(new Pose(50, 84), GPPpose)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(GPPpose, scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(scorePose, new Pose(50, 58))
        ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180)).build();

        Intake2_ = follower.pathBuilder().addPath(
                new BezierLine(new Pose(50, 60), new Pose(8.75, 60))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        turn1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(8.75, 60), new Pose(40, 60))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(40, 60), scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143)).build();

        turn2 = follower.pathBuilder().addPath(
                new BezierLine(scorePose,new Pose(25, 55)))
        .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(90)).build();
    }

    public void buildFarTwelve(){
        startPose = new Pose(56, 8.75);
        scorePose = new Pose(56,15);
        PPGpose = new Pose(12, 35);
        PGPpose = new Pose(12 ,60);
        
        Intake1 = follower.pathBuilder().addPath(
        new BezierCurve(
          startPose,
          new Pose(56.100, 35.000),
          PPGpose)
        ).setTangentHeadingInterpolation()
            .build();

        goToScore1 = follower.pathBuilder().addPath(
            new BezierLine(PPGpose, scorePose)
        ).setTangentHeadingInterpolation().setReversed().build();
        
        Intake2 = follower.pathBuilder().addPath(
            new BezierCurve(
                scorePose,
                new Pose(56.000, 59.000),
                PGPpose)
        ).setTangentHeadingInterpolation()
            .build();
        
        goToScore2 = follower.pathBuilder().addPath(
            new BezierLine(PGPpose, scorePose)
        ).setTangentHeadingInterpolation().setReversed()
            .build();
        
         openGate = follower.pathBuilder().addPath(
            new BezierLine(
                scorePose, 
                new Pose(11.000, 60.000))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(150))
                ))).build();

        goToScore3 = follower.pathBuilder().addPath(
            new BezierLine(
                new Pose(11.000, 60.000),
                scorePose)
        ).setTangentHeadingInterpolation().setReversed()
            .build();
    }

}
