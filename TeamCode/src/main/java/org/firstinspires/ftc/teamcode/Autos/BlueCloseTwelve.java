package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueCloseTwelve extends CommandOpMode {
    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence, AutoSequence2;
    Shooter shooter;
    Intake intake;
    Transfer transfer;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20,123, Math.toRadians(143.5)));
        Paths = new BluePaths(BluePaths.AutoType.CLOSE_TWELVE_NO_TURRET, follower);
        Paths.buildPaths();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.ShootPreloads),
                transfer.SpinIn.alongWith(intake.SpinIn),
                new WaitCommand(2500),
                transfer.StopTransfer,
                new FollowPathCommand(follower, Paths.Intake1),
                intake.StopIntake,
                new FollowPathCommand(follower, Paths.openGate),
                new FollowPathCommand(follower, Paths.goToScore1),
                new InstantCommand(() -> {intake.Spin(1); transfer.Spin(1);}),
                new WaitCommand(2500),
                new InstantCommand(() -> transfer.Spin(0)),
                new FollowPathCommand(follower, Paths.Intake2),
                new InstantCommand(() -> intake.Spin(0)),
                new FollowPathCommand(follower, Paths.goToScore2),
                new InstantCommand(() -> {intake.Spin(1); transfer.Spin(1);}),
                new WaitCommand(2500),
                new InstantCommand(() -> transfer.Spin(0)),
                new FollowPathCommand(follower, Paths.Intake3),
                new InstantCommand(() -> intake.Spin(0)),
                new FollowPathCommand(follower, Paths.goToScore3),
                new InstantCommand(() -> {intake.Spin(1); transfer.Spin(1);}),
                new WaitCommand(2500),
                new InstantCommand(() -> {transfer.Spin(0); intake.Spin(0);}),
                new FollowPathCommand(follower, Paths.move)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        follower.update();
        shooter.setTo(.5);
        super.run();
    }
}
