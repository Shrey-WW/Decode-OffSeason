package org.firstinspires.ftc.teamcode.Requirements;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class BluePaths {

    private final AutoType autoType;

    private final Follower follower;

    private GoBildaPinpointDriver pinpoint;

    public Pose startPose, Spike1, Spike2, Spike3, scorePose, gatePose;

    public PathChain Intake1, goToScore1, Intake2, goToScore2, Intake3, goToScore3, Intake4, goToScore4;
    public PathChain Intake2_, Intake1_, ShootPreloads, Intake4_;

    public PathChain fillerPath, fillerPath2, leave, openGate, openGate2;

    public BluePaths(AutoType type, Follower f){
        follower = f;
        autoType = type;
    }

    public void buildPaths(){
        if(autoType == AutoType.CLOSE_TWELVE_NO_TURRET) {
            buildCloseTwelveNoTurret();
        }
        else if(autoType == AutoType.FAR_TWELVE_NO_TURRET) {
            buildFarTwelveNoTurret();
        }
        else if(autoType == AutoType.EXODUS){
            buildExodus();
        }
        else if (autoType == AutoType.CLOSE_TWELVE){
            buildCloseTwelve();
        }
    }

    public void buildCloseTwelveNoTurret(){
        startPose = new Pose(20,123);
        Spike1 = new Pose(24, 85.5);
        gatePose = new Pose(18, 74);
        Spike2 = new Pose(20, 59);
        Spike3 = new Pose(20, 42);

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
                new BezierLine(Spike1, new Pose(55, 85.5))
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
                        new Pose(57, 42))
        ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(180))
                ))).build();

        Intake3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57, 42),
                        Spike3)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore3 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike3,
                        new Pose(55, 120)
                )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, .7, HeadingInterpolator.tangent.reverse()),
                new HeadingInterpolator.PiecewiseNode(.7, 1, HeadingInterpolator.constant(Math.toRadians(180))
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
                                new Pose(12.500, 35.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        goToScore1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.500, 35.000),

                                new Pose(55.500, 15.000)
                        )
                ).setTangentHeadingInterpolation().setReversed()
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

                                new Pose(25.000, 8.750)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Intake3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 8.750),

                                new Pose(12.000, 8.750)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fillerPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 13.000),

                                new Pose(25.000, 8.750)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fillerPath2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 8.750),

                                new Pose(12.000, 8.750)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        goToScore3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 8.750),
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
    public void buildExodus(){
        startPose = new Pose(19,123);
        Spike1 = new Pose(17, 84);
        gatePose = new Pose(19, 70);
        Spike2 = new Pose(18, 60);

        ShootPreloads = follower.pathBuilder().addPath(
                new BezierLine(startPose, new Pose(60, 83))
        ).setLinearHeadingInterpolation(Math.toRadians(144),Math.toRadians(-90)).build();

        Intake1 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(60, 83),
                        new Pose(57,57),
                        Spike2)
        ).setTangentHeadingInterpolation().build();

        openGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        Spike2,
                        new Pose(30, 72),
                        gatePose)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore1 = follower.pathBuilder().addPath(
                new BezierLine(gatePose, new Pose(60, 83))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();


        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(60, 83),
                        Spike1)
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        goToScore2 = follower.pathBuilder().addPath(
                new BezierLine(
                        Spike1,
                        new Pose(61, 83))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        openGate2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(60, 83),
                        new Pose(38,70),
                        gatePose)
        ).setTangentHeadingInterpolation().build();
        leave = follower.pathBuilder().addPath(
                new BezierLine(
                        gatePose,
                        new Pose(38,70))
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();
    }

    public void buildCloseTwelve() {
        startPose = new Pose(19, 121);
        Spike1 = new Pose(21, 84);
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
                        new Pose(38.5, 95)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        leave = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(38.5, 95),
                        new Pose(28, 95)
                )).setConstantHeadingInterpolation(Math.toRadians(180)).build();

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

