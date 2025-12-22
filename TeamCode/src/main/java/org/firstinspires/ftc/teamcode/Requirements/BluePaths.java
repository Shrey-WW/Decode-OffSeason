package org.firstinspires.ftc.teamcode.Requirements;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import java.io.File;

public class BluePaths {

    private final AutoType autoType;

    private final Follower follower;

    public Pose startPose, GPPpose, PGPpose, PPGpose, scorePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2, Intake3, goToScore3;
    public PathChain Intake2_, Intake1_, ShootPreloads;

    public PathChain fillerPath, fillerPath2, move, openGate;

    public enum AutoType {
        FAR_TWELVE, CLOSE_TWELVE, CLOSE_NINE, FAR_NINE, TEST, TEST2,
    }

    public BluePaths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if (autoType == AutoType.FAR_NINE){
            buildFarNine();
        }
        else if(autoType == AutoType.CLOSE_NINE) {
            buildCloseNine();
        }
        else if(autoType == AutoType.FAR_TWELVE) {
            buildFarTwelve();
        }
        else if(autoType == AutoType.CLOSE_TWELVE) {
            buildCloseTwelve();
        }
        else if(autoType == AutoType.TEST) {
            buildTest();
        }
        else if(autoType == AutoType.TEST2) {
            buildTest2();
        }
    }

    private void buildFarNine(){
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
                )).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
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
                        new HeadingInterpolator.PiecewiseNode(0, .6, HeadingInterpolator.constant(Math.toRadians(115))),
                        new HeadingInterpolator.PiecewiseNode(.6,1, HeadingInterpolator.linear(Math.toRadians(115), Math.toRadians(180))
                ))).build();

        Intake2_ = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(56.000, 60.000), PGPpose)
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierLine(PGPpose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();
        move = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(30,25)))
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
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

        fillerPath = follower.pathBuilder().addPath(
                new BezierLine(new Pose(8.75, 60), new Pose(40, 60))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(40, 60), scorePose)
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143)).build();

        fillerPath2 = follower.pathBuilder().addPath(
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
        
         openGate = follower.pathBuilder().addPath(
            new BezierLine(
                scorePose, 
                new Pose(12, 61))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent),
                        new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(150))
                ))).build();

        goToScore3 = follower.pathBuilder().addPath(
            new BezierLine(
                new Pose(12, 61),
                scorePose)
        ).setTangentHeadingInterpolation().setReversed()
            .build();
    }

    public void buildCloseTwelve(){
        startPose = new Pose(19,123);
        GPPpose = new Pose(18.5, 83);
        PGPpose = new Pose(12.5, 57);
        PPGpose = new Pose(12.5, 35);

        ShootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, new Pose(60, 83))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(180))
                ))).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(60, 83), GPPpose)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        GPPpose,
                        new Pose(23, 75),
                        new Pose(16, 75))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(16, 75), new Pose(60, 78))
                )
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(-148))
                        ))).build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60, 78),
                                new Pose(25, 57),
                                new Pose(12.5, 57))
                ).setTangentHeadingInterpolation().build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(12.5, 57),
                        new Pose(25, 57),
                        new Pose(60, 78))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(-115))
                ))).build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60, 78),
                                new Pose(40, 35),
                                new Pose(12.5, 35))
                ).setTangentHeadingInterpolation().build();

        goToScore3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12.5, 35), new Pose(60, 78))
                ).setTangentHeadingInterpolation().setReversed().build();

    }

    public void buildTest(){
        fillerPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.750),
                                new Pose(50.000, 9.500),
                                new Pose(12.000, 9.500)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fillerPath2 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(12.000, 9.500), new Pose(20.000, 9.500)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        move = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(20.000, 9.500), new Pose(12.000, 9.500)))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void buildTest2(){
        fillerPath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.750),
                                new Pose(20.000, 35.000),
                                new Pose(8.750, 25.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))
                .build();

        fillerPath2 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(8.750, 25.000), new Pose(8.750, 8)))
                .setTangentHeadingInterpolation()
                .build();
    }


}
