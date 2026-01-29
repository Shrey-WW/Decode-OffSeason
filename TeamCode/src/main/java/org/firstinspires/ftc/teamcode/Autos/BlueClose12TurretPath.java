package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.AutoType;
import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.Requirements.LaunchState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueClose12TurretPath extends CommandOpMode {
    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;
    Shooter shooter;
    Intake intake;
    Transfer transfer;

    public static LaunchState launchState;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19, 123, Math.toRadians(144)));
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        Paths = new BluePaths(AutoType.CLOSE_TWELVE, follower);
        Paths.buildPaths();
        launchState = LaunchState.SHOOTING;

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Paths.Intake1),
                        new WaitUntilCommand(() -> follower.atPose(new Pose(40.5, 62), 3, 3))
                                .andThen(new InstantCommand(() -> follower.setMaxPower(.6)))
                ),
                new InstantCommand(() -> follower.setMaxPower(1)),
                new FollowPathCommand(follower, Paths.openGate),
                new FollowPathCommand(follower, Paths.goToScore1),
                new FollowPathCommand(follower, Paths.openGate2),
                    new WaitCommand(200),
                new FollowPathCommand(follower, Paths.Intake2),
                new ParallelCommandGroup(
                        new WaitCommand(3000),
                        new InstantCommand(() -> intake.Spin(1))
                ),
                new FollowPathCommand(follower, Paths.goToScore2),
                new FollowPathCommand(follower, Paths.Intake3),
                new FollowPathCommand(follower, Paths.goToScore3),
                new FollowPathCommand(follower, Paths.leave)
        );

        SequentialCommandGroup Auto = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new FollowPathCommand(follower, Paths.Intake1),
                new FollowPathCommand(follower, Paths.openGate),
                new InstantCommand(() -> intake.Spin(1)),
                new FollowPathCommand(follower, Paths.Intake2)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        Pose cPose = follower.getPose();
        telemetry.addData("x", cPose.getX());
        telemetry.addData("y", cPose.getY());
        telemetry.addData("heading", cPose.getHeading());
        telemetry.update();
    }

}
