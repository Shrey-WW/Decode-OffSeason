package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.nio.file.Path;

public class BlueTwelvePathing extends CommandOpMode {
    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8.75, Math.PI/2));
        Paths = new BluePaths(BluePaths.AutoType.FAR_TWELVE, follower);
        Paths.buildPaths();

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.Intake1),
                new FollowPathCommand(follower, Paths.goToScore1),
                new FollowPathCommand(follower, Paths.Intake2),
                new FollowPathCommand(follower, Paths.goToScore2),
                new FollowPathCommand(follower, Paths.openGate),
                new FollowPathCommand(follower, Paths.goToScore3)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        follower.update();
        super.run();
    }
}
