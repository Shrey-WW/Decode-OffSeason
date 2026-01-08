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
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueClose extends CommandOpMode {
    private Follower follower;
    Limelight3A limelight;
    IMU imu;
    BluePaths Paths;
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    Turret turret;
    InstantCommand startIntake, stopIntake, reverseIntake, hoodUp;
    SequentialCommandGroup AutoSequence;

    @Override
    public void initialize(){
        super.reset();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19, 123, Math.toRadians(143.5)));
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        Paths = new BluePaths(BluePaths.AutoType.CLOSE_NINE, follower);
        Paths.buildPaths();

        startIntake = new InstantCommand(() -> intake.Spin(1));
        stopIntake = new InstantCommand(intake::PwrOff);
        reverseIntake = new InstantCommand(() -> intake.Spin(-1));
        hoodUp = new InstantCommand( () -> shooter.moveServo(0));

        AutoSequence = new SequentialCommandGroup(
                /// shooting preloads
                hoodUp,
                new FollowPathCommand(follower, Paths.ShootPreloads),
                transfer.open,
                new WaitCommand(500),
                startIntake,
                new WaitCommand(4000),
                /// intaking
                transfer.close,
                new FollowPathCommand(follower, Paths.Intake1),
                startIntake,
                new FollowPathCommand(follower, Paths.Intake1_),
                /// shooting
                new FollowPathCommand(follower, Paths.goToScore1),
                transfer.open,
                startIntake,
                new WaitCommand(4000),
                /// intaking
                transfer.close,
                new FollowPathCommand(follower, Paths.Intake2),
                startIntake,
                new FollowPathCommand(follower, Paths.Intake2_),
                /// shooting
                new FollowPathCommand(follower, Paths.goToScore2),
                transfer.open,
                startIntake,
                new WaitCommand(4000),
                new FollowPathCommand(follower, Paths.fillerPath2)
        );
        register(intake, shooter, transfer, turret);
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        super.run();
        shooter.setTo(.46);
        follower.update();
    }
}
