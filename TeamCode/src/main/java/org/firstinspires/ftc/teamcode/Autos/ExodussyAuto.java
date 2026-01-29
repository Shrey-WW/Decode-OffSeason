package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.AutoType;
import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class ExodussyAuto extends CommandOpMode {
    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;
    Intake intake;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19, 123, Math.toRadians(143.5)));
        Paths = new BluePaths(AutoType.EXODUS, follower);
        Paths.buildPaths();
        intake = new Intake(hardwareMap);

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new FollowPathCommand(follower, Paths.Intake1),
                new FollowPathCommand(follower, Paths.openGate),
                new FollowPathCommand(follower, Paths.goToScore1),
                new FollowPathCommand(follower, Paths.Intake2),
                new FollowPathCommand(follower, Paths.goToScore2),
                new FollowPathCommand(follower, Paths.openGate2),
                new WaitCommand(11500),
                new FollowPathCommand(follower, Paths.leave)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        follower.update();
        super.run();
    }
}
