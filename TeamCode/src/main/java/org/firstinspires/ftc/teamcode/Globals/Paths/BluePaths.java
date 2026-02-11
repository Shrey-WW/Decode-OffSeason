package org.firstinspires.ftc.teamcode.Globals.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.Globals.Paradigms.Paths;
import org.firstinspires.ftc.teamcode.Globals.States.AutoType;

public class BluePaths extends Paths {

    public BluePaths(AutoType type, Follower f){
        super(type, f);
    }

    public void buildPaths(){
        super.buildPaths();
    }

    public void buildCloseTwelveNoTurret(){
        startPose = new Pose(19, 121);
        Spike1 = new Pose(22, 85.5);
        gatePose = new Pose(20, 72);
        Spike2 = new Pose(21, 58);
        Spike3 = new Pose(21, 38);

        ShootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, new Pose(55, 85.5))
        ).setConstantHeadingInterpolation(Math.toRadians(130)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(55, 85.5), Spike1)
        ).setTangentHeadingInterpolation().build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        Spike1,
                        new Pose(35, 80),
                        gatePose)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(gatePose, new Pose(55, 85.5))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(131))
                ))).build();

        Intake1_ = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(55, 85.5),
                        new Pose(55, 59))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(55, 59),
                        Spike2)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike2,
                        new Pose(55, 85.5))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(128))
                ))).build();

        Intake2_ = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(55, 85.5),
                        new Pose(57, 38))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(180))
                ))).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57, 38),
                        Spike3)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore3 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike3,
                        new Pose(55, 115)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(165))
                ))).build();

    }

    public void buildFarTwelveNoTurret(){
        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.500, 8.000),

                                new Pose(55.500, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))

                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.500, 15.000),
                                new Pose(55.500, 35.000),
                                new Pose(11, 36.500)
                        )
                ).setTangentHeadingInterpolation()
                .build();



        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.500, 35.000),

                                new Pose(55.500, 15.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(115))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.500, 15.000),
                                new Pose(55.500, 58.000),
                                new Pose(12.500, 58.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        goToScore2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.500, 58.000),
                                new Pose(55.500, 15.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Intake1_ = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.500, 15.000),
                                new Pose(12.000, 14.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Intake2_ = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 14.000),

                                new Pose(25.000, 9)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 9),

                                new Pose(9, 9)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fillerPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9, 9),

                                new Pose(25.000, 9)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fillerPath2 = follower.pathBuilder().addPath(
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

    public void buildCloseTwelve() {
        startPose = new Pose(19, 121);
        Spike1 = new Pose(19, 84);
        Spike2 = new Pose(21, 55);
        Spike3 = new Pose(21, 42);

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(55.500, 85.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 85.5),
                        new Pose(55.5, 59),
                        Spike2
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        Spike2,
                        new Pose(24.5, 61),
                        new Pose(20, 68)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(18.5, 68),
                        new Pose(48,70),
                        new Pose(55.5, 85.5)
                )).setTangentHeadingInterpolation().setReversed().build();

        openGate2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 85.5),
                        new Pose(48, 70),
                        new Pose(20, 68)
                )).setTangentHeadingInterpolation().build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(20, 68),
                        new Pose(23, 62),
                        new Pose(15.5, 56)
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(16, 56),
                        new Pose(42, 67.5),
                        new Pose(58, 84)
                )).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(58, 84),
                        Spike1
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore3 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike1,
                        new Pose(38.5, 100)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

    }

    public void buildAutoTheory12(){
        startPose = new Pose(19, 121);
        gatePose = new Pose(23, 74);

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(47.75, 100.7)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), 2.44)
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(23, 74)
                        )
                ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(180))
                ))).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(23, 74),
                        new Pose(63.2, 84)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), 2.46).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(63.2, 84),
                        new Pose(27.7, 53.8)
                )
        ).setConstantHeadingInterpolation(-2.49).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(27.7, 53.8),
                        new Pose(62, 91.5)
                )
        ).setLinearHeadingInterpolation(-2.49, 2.58).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(62, 91.5),
                        new Pose(59.4, 35),
                        new Pose(59.4, 35),
                        new Pose(22, 35)
                )).setTangentHeadingInterpolation().build();
        goToScore3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(22, 35),
                        new Pose(57, 115)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .8, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.8, 1, HeadingInterpolator.constant(Math.toRadians(180))
                ))).build();
    }

    public void buildCloseFifteen(){
        startPose = new Pose(19, 121);
        Spike1 = new Pose(23, 84);
        Spike2 = new Pose(21, 55);
        Spike3 = new Pose(20, 42);

        ShootPreloads = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new Pose(55.500, 85.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 85.5),
                        new Pose(55.5, 59),
                        Spike2
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        Spike2,
                        new Pose(27, 61),
                        new Pose(19, 68)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(19, 68),
                        new Pose(48,70),
                        new Pose(55.5, 85.5)
                )).setTangentHeadingInterpolation().setReversed().build();

        openGate2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(55.5, 85.5),
                        new Pose(48, 70),
                        new Pose(24, 68)
                )).setTangentHeadingInterpolation().setTimeoutConstraint(0).build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(24, 68),
                        new Pose(25, 62),
                        new Pose(14.5, 56)
                )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(14.5, 56),
                        new Pose(42, 67.5),
                        new Pose(58, 84)
                )).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180)).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(58, 84),
                        Spike1
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore3 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike1,
                        new Pose(58, 84)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Intake4_ = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(58, 84),
                        new Pose(45, 42)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Intake4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(45, 42),
                        Spike3
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore4 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike3,
                        new Pose(38.5, 94)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        leave = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(38.5, 95),
                        new Pose(28, 94)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();
    }

}

