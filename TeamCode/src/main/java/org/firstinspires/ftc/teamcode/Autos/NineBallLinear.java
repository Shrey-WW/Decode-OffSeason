package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class NineBallLinear extends CommandOpMode{
    private Follower follower;
    Limelight3A limelight;
    IMU imu;
    Paths paths;
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    Turret turret;
    private static final int TICKS_PER_REV = 2403;
    InstantCommand startIntake, stopIntake, reverseIntake, startFlywheel, restFlywheel, hoodUp;
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
        follower.setStartingPose(new Pose(56, 8.75, Math.PI/2));
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        paths = new Paths(Paths.AutoType.LINEAR_NINE, follower);
        paths.buildPaths();

        startIntake = new InstantCommand(() -> intake.Spin(1));
        stopIntake = new InstantCommand(intake::PwrOff);
        reverseIntake = new InstantCommand(() -> intake.Spin(-1));
        startFlywheel = new InstantCommand(() -> shooter.setTo(.68));
        restFlywheel = new InstantCommand( () -> shooter.setTo(.3));
        hoodUp = new InstantCommand( () -> shooter.moveServo(.62));


        AutoSequence = new SequentialCommandGroup(
                hoodUp,
                new InstantCommand(()-> shooter.setTo(.61)),
                transfer.close,
                new WaitCommand(2500),
                startIntake,
                transfer.open,
                transfer.open,
                new WaitCommand(5000),
                new FollowPathCommand(follower, paths.Intake1),
                transfer.close,
                new FollowPathCommand(follower, paths.Intake1_),
                new FollowPathCommand(follower, paths.goToScore1),
                new InstantCommand(()-> autoAlign(limelight.getLatestResult().getTx())),
                transfer.open,
                new WaitCommand(5000),
                new FollowPathCommand(follower, paths.Intake2),
                transfer.close,
                new FollowPathCommand(follower, paths.Intake2_),
                new FollowPathCommand(follower, paths.goToScore2),
                transfer.open,
                new WaitCommand(5000)
        );
        register(intake, shooter, transfer, turret);
        schedule(AutoSequence);

    }

    @Override
    public void run(){
        super.run();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        follower.update();
    }

    private void autoAlign(double Tx){
        int cPos = turret.getPos();
        turret.goToPos((int) (cPos - TICKS_PER_REV * Tx/360));
    }
}
