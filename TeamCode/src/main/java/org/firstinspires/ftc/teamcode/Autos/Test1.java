package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous
public class Test1 extends CommandOpMode {
    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8.75, Math.PI));
        Paths = new BluePaths(BluePaths.AutoType.TEST, follower);
        Paths.buildPaths();

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.fillerPath),
                new FollowPathCommand(follower, Paths.fillerPath2),
                new FollowPathCommand(follower, Paths.move)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        follower.update();
        super.run();
    }
}
