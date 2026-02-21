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
public class ElliotClose extends CommandOpMode {
    private Follower follower;

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
    public PathChain Path11;
    public PathChain Path12;
    public PathChain Path13;


    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(17.864, 117.345));

        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(17.864, 117.345),
                                new Pose(61.855, 73.662)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(61.855, 73.662),
                                new Pose(19.510, 57.945)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.510, 57.945),
                                new Pose(61.879, 73.712)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(61.879, 73.712),
                                new Pose(16.477, 65.312)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(16.477, 65.312),
                                new Pose(12.220, 57.202)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.220, 57.202),
                                new Pose(61.945, 73.367)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(61.945, 73.367),
                                new Pose(16.519, 65.475)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(16.519, 65.475),
                                new Pose(12.182, 57.145)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.182, 57.145),
                                new Pose(61.804, 73.418)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(61.804, 73.418),
                                new Pose(19.749, 85.490)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.749, 85.490),
                                new Pose(61.758, 73.484)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path12 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(61.758, 73.484),
                                new Pose(58.889, 35.367),
                                new Pose(18.846, 34.754)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setReversed()
                .build();

        Path13 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(18.846, 34.754),
                                new Pose(60.084, 108.004)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        SequentialCommandGroup auto = new SequentialCommandGroup(
                new FollowPathCommand(follower, Path1),
                new FollowPathCommand(follower, Path2),
                new FollowPathCommand(follower, Path3),
                new FollowPathCommand(follower, Path4),
                new FollowPathCommand(follower, Path5),
                new FollowPathCommand(follower, Path6),
                new FollowPathCommand(follower, Path7),
                new FollowPathCommand(follower, Path8),
                new FollowPathCommand(follower, Path9),
                new FollowPathCommand(follower, Path10),
                new FollowPathCommand(follower, Path11),
                new FollowPathCommand(follower, Path12),
                new FollowPathCommand(follower, Path13)
        );
        schedule(auto);
    }

    @Override
    public void run(){
        super.run();
        follower.update();
    }
}
