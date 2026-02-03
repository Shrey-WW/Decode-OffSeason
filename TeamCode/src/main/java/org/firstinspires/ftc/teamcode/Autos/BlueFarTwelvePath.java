package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Globals.AutoType;
import org.firstinspires.ftc.teamcode.Globals.BluePaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class BlueFarTwelvePath extends CommandOpMode {
    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;
    Intake intake;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8.75, Math.PI/2));
        Paths = new BluePaths(AutoType.FAR_TWELVE_NO_TURRET, follower);
        Paths.buildPaths();
        intake = new Intake(hardwareMap);

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new FollowPathCommand(follower, Paths.Intake1_),
                new FollowPathCommand(follower, Paths.Intake2_),
                new FollowPathCommand(follower, Paths.Intake3),
                new FollowPathCommand(follower, Paths.fillerPath),
                new FollowPathCommand(follower, Paths.fillerPath2),
                new FollowPathCommand(follower, Paths.goToScore3),
                new FollowPathCommand(follower, Paths.Intake1),
                new FollowPathCommand(follower, Paths.goToScore1),
                new FollowPathCommand(follower, Paths.Intake2),
                new FollowPathCommand(follower, Paths.goToScore2),
                new FollowPathCommand(follower, Paths.leave)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        follower.update();
        intake.Spin(1);
        super.run();
    }
}
