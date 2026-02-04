package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Globals.AutoStates;
import org.firstinspires.ftc.teamcode.Globals.AutoType;
import org.firstinspires.ftc.teamcode.Globals.BluePaths;
import org.firstinspires.ftc.teamcode.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.Globals.LaunchState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (group = "a. New")
public class Blue12close extends CommandOpMode {

    private Follower follower;
    private IMU imu;
    private BluePaths Paths;
    private  SequentialCommandGroup AutoSequence;
    private Limelight3A limelight;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private AutoStates states;


    @Override
    public void initialize(){
        states = new AutoStates();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20, 123, Math.toRadians(143.5)));

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        limelight.pipelineSwitch(0);
        limelight.start();

        Paths = new BluePaths(AutoType.CLOSE_TWELVE_NO_TURRET, follower);
        Paths.buildPaths();

        AutoSequence = new SequentialCommandGroup(
                /// Shooting preloads
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new AutoShootingCMD(shooter, transfer, intake, limelight),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake1),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore1),
                new AutoShootingCMD(shooter, transfer, intake, limelight),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake1_),
                new InstantCommand(() -> follower.setMaxPower(.5)),
                new FollowPathCommand(follower, Paths.Intake2),
                new InstantCommand(() -> follower.setMaxPower(1)),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore2),
                new AutoShootingCMD(shooter, transfer, intake, limelight),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake2_),
                new FollowPathCommand(follower, Paths.Intake3),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore3),
                new AutoShootingCMD(shooter, transfer, intake, limelight),
                new InstantCommand(() -> states.launchstate = LaunchState.END)
        );

        schedule(AutoSequence.alongWith(new InstantCommand(() -> transfer.setPos(0))));
    }

    @Override
    public void run() {
        super.run();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        follower.update();
        if (states.launchstate == LaunchState.IDLE)
            shooter.setTo(.35);
        else if (states.launchstate == LaunchState.END){
            shooter.setTo(.2);
        }

        Pose cPose = follower.getPose();
        telemetry.addData("x", cPose.getX());
        telemetry.addData("y", cPose.getY());
        telemetry.addData("heading", cPose.getHeading());
        telemetry.addData("did da ball shoot or nah?", AutoShootingCMD.numBallsShot);
        telemetry.update();
    }

}
