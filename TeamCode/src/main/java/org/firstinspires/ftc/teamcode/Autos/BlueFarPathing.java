package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueFarPathing extends CommandOpMode {
    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;

    @Override
    public void initialize(){
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8.75, Math.PI/2));

        Paths = new BluePaths(BluePaths.AutoType.FAR_NINE, follower);
        Paths.buildPaths();

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new FollowPathCommand(follower, Paths.Intake1),
                new FollowPathCommand(follower, Paths.Intake1_),
                new FollowPathCommand(follower, Paths.goToScore1),
                new FollowPathCommand(follower, Paths.Intake2),
                new FollowPathCommand(follower, Paths.Intake2_),
                new FollowPathCommand(follower, Paths.goToScore2)
        );
        schedule(AutoSequence);

    }

    @Override
    public void run(){
        super.run();
        follower.update();
    }
}
