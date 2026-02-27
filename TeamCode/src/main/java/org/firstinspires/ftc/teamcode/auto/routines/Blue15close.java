package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.AutoType;

@Autonomous
public class Blue15close extends AutoBase {
    @Override
    public void initialize(){
        autoType = AutoType.CLOSE_15;
        startingPose = new Pose(19, 121, Math.toRadians(144));
        super.initialize();

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.shootPreloads),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 10000),
                new FollowPathCommand(follower, paths.intake1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 10000),
                new FollowPathCommand(follower, paths.intake2),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 10000),
                new FollowPathCommand(follower, paths.intake3),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 10000),
                new FollowPathCommand(follower, paths.intake4),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 10000)
                );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        OdomTracking();
        updateShooter();
        super.run();
    }
}
