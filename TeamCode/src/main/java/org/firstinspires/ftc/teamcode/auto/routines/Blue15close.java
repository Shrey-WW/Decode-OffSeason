package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.AutoState;
import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.constants.LaunchState;

@Autonomous
public class Blue15close extends AutoBase {
    @Override
    public void initialize(){
        autoType = AutoType.CLOSE_15;
        startingPose = new Pose(19, 121, 2.4065);
        super.initialize();

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.shootPreloads),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 1900),
                new FollowPathCommand(follower, paths.intake1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 1900),
                new FollowPathCommand(follower, paths.intake2),
                new WaitCommand(2000),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                new FollowPathCommand(follower, paths.goToScore2),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 1900),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.IDLE),
                new FollowPathCommand(follower, paths.intake3),
                new WaitCommand(2000),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                new FollowPathCommand(follower, paths.goToScore3),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 1900),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.IDLE),
                new FollowPathCommand(follower, paths.intake4),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 1900),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.END)
                );
        schedule(AutoSequence, new InstantCommand(() -> shooter.moveServo(.8)));
    }

    @Override
    public void run(){
        ARC();
        updateShooter();
        super.run();
    }
}
