package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
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
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.shootPreloads),
                        new WaitUntilCommand(() -> follower.atPose(new Pose(39.5, 100), 5, 5))
                                .andThen(new AutoShootingCMD(1200, shooter, transfer, intake, turret, limelight)
                                        .alongWith(new InstantCommand(() -> follower.setMaxPower(.75))))
                ),
                new InstantCommand(() -> follower.setMaxPower(1)),
                new FollowPathCommand(follower, paths.intake1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 2000),
                new FollowPathCommand(follower, paths.intake2),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 2000),
                new FollowPathCommand(follower, paths.intake3),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 2000),
                new FollowPathCommand(follower, paths.intake4),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 2000),
                new InstantCommand(() -> turret.TurnTo(0))
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
