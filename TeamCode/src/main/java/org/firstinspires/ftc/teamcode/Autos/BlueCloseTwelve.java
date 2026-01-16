package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.AutoType;
import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueCloseTwelve extends CommandOpMode {
    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;
    Shooter shooter;
    Intake intake;
    Transfer transfer;
    private double x = .45;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20,123, Math.toRadians(143.5)));
        Paths = new BluePaths(AutoType.CLOSE_TWELVE_NO_TURRET, follower);
        Paths.buildPaths();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);

        AutoSequence = new SequentialCommandGroup(
                /// initial commands
                new InstantCommand(() -> transfer.setPos(.2)),
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new InstantCommand(() -> transfer.setPos(0)),
                /// Shooting preloads
                transfer.SpinIn.alongWith(intake.SpinIn),
                new WaitCommand(500),
                new InstantCommand(() -> x = .65),
                new WaitCommand(2000),
                new InstantCommand(() -> x = .45),
                transfer.StopTransfer,
                new InstantCommand(() -> transfer.setPos(.2)),
                /// First Intake sequence
                new FollowPathCommand(follower, Paths.Intake1),
                new FollowPathCommand(follower, Paths.openGate),
                new WaitCommand(400),
                new FollowPathCommand(follower, Paths.goToScore1),
                /// Shooting
                new InstantCommand(() -> transfer.setPos(0)),
                new InstantCommand(() -> {intake.Spin(1); transfer.Spin(1);}),
                new WaitCommand(500),
                new InstantCommand(() -> x = .65),
                new WaitCommand(2000),
                new InstantCommand(() -> x = .45),
                new InstantCommand(() -> transfer.setPos(.2)),
                new InstantCommand(() -> transfer.Spin(0)),
                /// Second Intake sequence
                new FollowPathCommand(follower, Paths.Intake2),
                new FollowPathCommand(follower, Paths.goToScore2),
                new InstantCommand(() -> transfer.setPos(0)),
                /// Shooting
                new InstantCommand(() -> {intake.Spin(1); transfer.Spin(1);}),
                new WaitCommand(500),
                new InstantCommand(() -> x = .65),
                new WaitCommand(2000),
                new InstantCommand(() -> x = .45),
                new InstantCommand(() -> transfer.setPos(.2)),
                /// Third Intake sequence
                new FollowPathCommand(follower, Paths.Intake2_),
                new FollowPathCommand(follower, Paths.Intake3),
                new FollowPathCommand(follower, Paths.goToScore3),
                new InstantCommand(() -> transfer.setPos(0)),
                /// Shooting
                new InstantCommand(() -> {intake.Spin(1); transfer.Spin(1);}),
                new WaitCommand(500),
                new InstantCommand(() -> x = .65),
                new WaitCommand(2000),
                new InstantCommand(() -> x = 0),
                new FollowPathCommand(follower, Paths.move)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        follower.update();
        shooter.setTo(x);
        super.run();
    }
}
