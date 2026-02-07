package org.firstinspires.ftc.teamcode.Globals.Paradigms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Globals.Paths.BluePaths;
import org.firstinspires.ftc.teamcode.Globals.Paths.RedPaths;
import org.firstinspires.ftc.teamcode.Globals.States.Alliance;
import org.firstinspires.ftc.teamcode.Globals.States.AutoType;
import org.firstinspires.ftc.teamcode.SolversLib.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.Globals.States.AutoStates;
import org.firstinspires.ftc.teamcode.Globals.States.LaunchState;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class AutoBase extends CommandOpMode {

    /// Subsystems
    protected Intake intake;
    protected Shooter shooter;
    protected Transfer transfer;
    protected Turret turret;
    protected Limelight3A limelight;
    protected Follower follower;
    protected IMU imu;

    /// Class specifics
    /// protected
    protected AutoType autoType;
    protected Alliance alliance;
    protected Paths paths;
    protected Pose startingPose;
    protected SequentialCommandGroup AutoSequence;
    public AutoStates AutoState;
    protected double GoalX, GoalY;

    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        AutoState.launchstate = LaunchState.IDLE;

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        if (alliance == Alliance.RED){
            limelight.pipelineSwitch(1);
            GoalX = 132.5;
            GoalY = 135;
            paths = new RedPaths(autoType, follower);
        }
        else {
            limelight.pipelineSwitch(0);
            GoalX = 13;
            GoalY = 135;
            paths = new BluePaths(autoType, follower);

        }
        paths.buildPaths();
        limelight.start();
    }

    public void run(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        if (AutoStates.launchstate == LaunchState.IDLE)
            shooter.setTo(.35);
        else if (AutoStates.launchstate == LaunchState.END){
            shooter.setTo(.2);
            turret.PIDto(0);
        }

        Pose cPose = follower.getPose();
        telemetry.addData("x", cPose.getX());
        telemetry.addData("y", cPose.getY());
        telemetry.addData("heading", cPose.getHeading());
        telemetry.addData("turretheading", turret.getTurretHeadingDeg());
        telemetry.addData("did da ball shoot or nah?", AutoShootingCMD.numBallsShot);

        telemetry.update();
        follower.update();
        super.run();
    }

    public void ARC(){
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            double RadianTx = -Math.toRadians(Tx);
            double targetPosition = RadianTx * 2 + turret.getPos();
            if (targetPosition >= 4.5 || targetPosition <= -3.3) {
                turret.pwrOff();
            } else {
                turret.PIDto(targetPosition);
            }
        }

        if (turret.getPos() <= -2.7 && turret.getPos() >= 4.5){
            turret.pwrOff();
        }
    }


}
