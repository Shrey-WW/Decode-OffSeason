package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class ElliotFar extends CommandOpMode {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain bennettsaddition;

    private Follower follower;


    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),
                                new Pose(53.943, 18.910)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.943, 18.910),
                                new Pose(43.690, 37.632),
                                new Pose(17.771, 35.927)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(17.771, 35.927),
                                new Pose(53.943, 18.822)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.943, 18.822),
                                new Pose(32.722, 7.688),
                                new Pose(8.868, 9)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.868, 9),
                                new Pose(19.138, 9)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.138, 9),
                                new Pose(8.782, 9)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(8.782, 9),
                                new Pose(55.875, 12)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.875, 12),
                                new Pose(16.335, 12.713),
                                new Pose(9.5, 24.332)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.5, 24.332),
                                new Pose(53.952, 18.818)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.952, 18.818),
                                new Pose(16.338, 12.614),
                                new Pose(9.5, 24.363)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();

        PathChain Path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.5, 24.363),
                                new Pose(53.952, 18.818)
                        )
                )
                .setTangentHeadingInterpolation().setReversed()
                .build();
        bennettsaddition = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.875 ,12 ),
                                new Pose(10, 52.607),
                                new Pose(9, 9)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();



        SequentialCommandGroup auto = new SequentialCommandGroup(
                new FollowPathCommand(follower, Path1),
                new FollowPathCommand(follower, Path2),
                new FollowPathCommand(follower, Path3),
                new FollowPathCommand(follower, bennettsaddition),
                new FollowPathCommand(follower, Path7),
                new FollowPathCommand(follower, Path8),
                new FollowPathCommand(follower, Path9),
                new FollowPathCommand(follower, Path10),
                new FollowPathCommand(follower, Path11)
        );
        schedule(auto);
    }

    @Override
    public void run(){
        super.run();
        follower.update();
    }
}
