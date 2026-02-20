package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.constants.AutoType;


@Autonomous
public class Blue12Theory extends AutoBase {

    @Override
    public void initialize(){
        autoType = AutoType.AutoTheory12;
        startingPose = new Pose(19, 121, Math.toRadians(144));
        super.initialize();

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.ShootPreloads),
                new FollowPathCommand(follower, paths.Intake1),
                new FollowPathCommand(follower, paths.goToScore1),
                new FollowPathCommand(follower, paths.Intake2),
                new FollowPathCommand(follower, paths.goToScore2),
                new FollowPathCommand(follower, paths.Intake3),
                new FollowPathCommand(follower, paths.goToScore3)
        );
    }


    @Override
    public void run(){
        super.run();
    }
}
