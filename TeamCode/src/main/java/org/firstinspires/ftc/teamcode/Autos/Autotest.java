package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Autotest extends OpMode {
    private Follower follower;
    PathChain Intake1, goToScore1, Intake2, Intake2_, goToScore2;
    @Override
    public void init(){

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(56, 8.75, Math.PI/2));
        Pose startPose = new Pose(56, 8.75,Math.PI/2);
        Pose scorePose = new Pose(56,15, Math.PI/2);
        Pose PPGpose = new Pose(16,37.5, Math.PI);
        Pose PGPpose = new Pose(16,60, Math.PI);
        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(56.000, 37.5),
                                PPGpose
                        )).setTangentHeadingInterpolation()
                .build();
        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(PPGpose, scorePose))
                .setTangentHeadingInterpolation().setReversed()
                .build();
        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(scorePose, new Pose(56, 60, Math.toRadians(180))))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.constant(Math.toRadians(90))),
                        new HeadingInterpolator.PiecewiseNode(.7,1, HeadingInterpolator.linear(Math.toRadians(90), Math.toRadians(180))
                        ))).build();
        Intake2_ = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(56.000, 60.000, Math.toRadians(180)), PGPpose)
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        goToScore2 = follower.pathBuilder().addPath(
                        new BezierLine(PGPpose, scorePose)
                ).setTangentHeadingInterpolation().setReversed()
                .build();
    }

    @Override
    public void start(){
        follower.followPath(Intake1);
    }

    @Override
    public void loop(){
        follower.update();

    }

}
