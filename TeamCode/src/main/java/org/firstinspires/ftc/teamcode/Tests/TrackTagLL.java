package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp (group = "tests")
public class TrackTagLL extends CommandOpMode {
    private double goalX = 132.5;
    private int goalY = 135;
    private static final double TicksPerRev = 19;
    private static final double StoppingThreshold = 5.5;
    public static double kV = 0;
    private Limelight3A limelight;
    private IMU imu;
    Turret turret;
    Follower follower;

    @Override
    public void initialize(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        turret = new Turret(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(125, 135, 0));
        follower.startTeleopDrive();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        register(turret);
        limelight.start();
    }

    @Override
    public void run(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
//        double vOmega = follower.getAngularVelocity();
//        double requiredTurretSpeed = -vOmega;
//
//        double feedforwardPower = requiredTurretSpeed * kV;

        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            telemetry.addData("Tx", Tx);
            double RadianTx = -Math.toRadians(Tx);
            double targetPosition = RadianTx + turret.getPos();
            telemetry.addData("target pos", targetPosition);
            turret.PIDto(targetPosition);
        }

//        telemetry.addData("ffpwr", feedforwardPower);
//        turret.setTo(feedforwardPower);
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        telemetry.addData("current pos", turret.getPos());
        telemetry.update();
        super.run();
    }


}
