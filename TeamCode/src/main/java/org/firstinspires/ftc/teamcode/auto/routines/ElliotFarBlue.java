package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.constants.AutoType;


@Autonomous
public class ElliotFarBlue extends AutoBase {


    @Override
    public void initialize(){
        autoType = AutoType.ELLIOT_FAR;
        startingPose = new Pose(56.000, 8.000, Math.toRadians(90));
        super.initialize();


        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.shootPreloads),
                new FollowPathCommand(follower, paths.intake1),
                new FollowPathCommand(follower, paths.intake2),
                new FollowPathCommand(follower, paths.intake3),
                new FollowPathCommand(follower, paths.intake4)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        super.run();
    }
}
