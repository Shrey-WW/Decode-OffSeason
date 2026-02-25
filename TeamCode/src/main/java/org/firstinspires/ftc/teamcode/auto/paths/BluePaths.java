package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.constants.AutoType;

public class BluePaths extends Paths {

    public BluePaths(AutoType autoType, Follower follower) {
        super(autoType, follower);
    }

    public void buildPaths() {
        super.buildPaths();
    }

    public void buildCloseTwelveNoTurret() {
        startPose = new Pose(19, 121);
        spike1 = new Pose(22, 85.5);
        gatePose = new Pose(20, 72);
        spike2 = new Pose(21, 58);
        spike3 = new Pose(21, 38);

        shootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, new Pose(55, 85.5))
        ).setConstantHeadingInterpolation(Math.toRadians(130)).build();

        intake1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(55, 85.5), spike1)
        ).setTangentHeadingInterpolation().build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        spike1,
                        new Pose(35, 80),
                        gatePose)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(gatePose, new Pose(55, 85.5))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(131))
                ))).build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55, 85.5), new Pose(55, 59)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Pose(55, 59), spike2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                spike2,
                                new Pose(55, 85.5))
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(128))
                        ))).build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55, 85.5), new Pose(57, 38)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(180))
                        )))
                .addPath(new BezierLine(new Pose(57, 38), spike3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                spike3,
                                new Pose(55, 115)
                        )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                        new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(165))
                        ))).build();
    }

    public void buildFarTwelveNoTurret() {
        shootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.500, 8.000),
                                new Pose(55.500, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .build();

        intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.500, 15.000),
                                new Pose(55.500, 35.000),
                                new Pose(11, 36.500)
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(12.500, 35.000),
                                new Pose(55.500, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(115))
                .build();

        intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.500, 15.000),
                                new Pose(55.500, 58.000),
                                new Pose(12.500, 58.000)
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(12.500, 58.000),
                                new Pose(55.500, 15.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55.500, 15.000), new Pose(12.000, 14.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Pose(12.000, 14.000), new Pose(25.000, 9)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Pose(25.000, 9), new Pose(9, 9)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeSweep1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9, 9),
                                new Pose(25.000, 9)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeSweep2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 9),
                                new Pose(9, 8.75)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToScore3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9, 9),
                                new Pose(55.500, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                .build();

        leave = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(55.500, 15.000),
                        new Pose(55.500, 25)
                )).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180)).build();
    }

    public void buildCloseFifteen() {
        startPose = new Pose(19, 121);
        spike1 = new Pose(22, 84);
        spike2 = new Pose(22, 51);
        spike3 = new Pose(22, 42);

        shootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(55.500, 84)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 84),
                        new Pose(55.5, 59),
                        spike2
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        spike2,
                        new Pose(32, 51),
                        new Pose(32, 51),
                        new Pose(23, 65)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(23, 65),
                        new Pose(43, 70),
                        new Pose(55.5, 84)
                )).setTangentHeadingInterpolation().setReversed().build();

        openGate2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 84),
                        new Pose(55.5, 59),
                        new Pose(23.5, 65)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(23.5, 65),
                        new Pose(33, 65),
                        new Pose(15, 50)
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .addPath(
                        new BezierCurve(
                                new Pose(15, 50),
                                new Pose(55.5, 84)
                        )).setTangentHeadingInterpolation().setReversed().build();

        openGate3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 84),
                        new Pose(55.5, 59),
                        new Pose(23.5, 65)
                )).setTangentHeadingInterpolation().setTimeoutConstraint(0).build();

        intake3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(23.5, 65),
                        new Pose(33, 65),
                        new Pose(15, 50)
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .addPath(
                        new BezierLine(
                                new Pose(15, 50),
                                new Pose(55.5, 84)
                        )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .3, HeadingInterpolator.linear(Math.toRadians(130), Math.toRadians(180))),
                        new HeadingInterpolator.PiecewiseNode(.3, 1, HeadingInterpolator.constant(Math.toRadians(180))
                        ))).build();

        intake4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(55.5, 84),
                        spike1
                )).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                spike1,
                                new Pose(48, 110)
                        )).setTangentHeadingInterpolation().setReversed().build();
    }

    public void buildElliotFar(){
        startPose = new Pose(56, 8);
        spike1 = new Pose(17.7, 36);
        Pose scorePose = new Pose(55, 17);

        shootPreloads = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();

        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(50, 36),
                                spike1
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                spike1,
                                scorePose
                        )
                ).setTangentHeadingInterpolation().setReversed().build();

        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(6.7, 40.6),
                                new Pose(9, 9)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, .65, HeadingInterpolator.linear(goToScore1.getFinalHeadingGoal(), Math.toRadians(270))),
                        new HeadingInterpolator.PiecewiseNode(.65, 1, HeadingInterpolator.constant(Math.toRadians(270))
                        )))
                .addPath(
                        new BezierLine(
                                new Pose(9, 9),
                                scorePose
                        )
                ).setTangentHeadingInterpolation().setReversed().build();

        intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(9.5, 24.3)
                        )
                )
                .setLinearHeadingInterpolation(intake2.getFinalHeadingGoal(), Math.toRadians(180))
                .addPath(new BezierCurve(
                                new Pose(9.5, 24.3),
                                new Pose(26.000, 16.000),
                                new Pose(26.000, 16.000),
                                new Pose(26.000, 16.000),
                                new Pose(9.500, 12.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(9.5, 12),
                                scorePose
                        )
                ).setTangentHeadingInterpolation().setReversed().build();

        intake4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(9.5, 24.3)
                        )
                )
                .setLinearHeadingInterpolation(intake3.getFinalHeadingGoal(), Math.toRadians(180))
                .addPath(new BezierCurve(
                                new Pose(9.5, 24.3),
                                new Pose(26.000, 16.000),
                                new Pose(26.000, 16.000),
                                new Pose(26.000, 16.000),
                                new Pose(9.500, 12.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(9.5, 12),
                                scorePose
                        )
                ).setTangentHeadingInterpolation().setReversed().build();
    }
}