package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.AutoState;
import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.constants.LaunchState;


@Autonomous (group = "a. New")
public class Blue12close extends AutoBase {

    @Override
    public void initialize(){
        autoType = AutoType.CLOSE_TWELVE_NO_TURRET;
        startingPose = new Pose(19, 121, Math.toRadians(144));
        super.initialize();

        AutoSequence = new SequentialCommandGroup(
                /// Shooting preloads
                new FollowPathCommand(follower, paths.shootPreloads),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, paths.intake1),
                new FollowPathCommand(follower, paths.openGate),
                /// Shooting
                new FollowPathCommand(follower, paths.goToScore1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, paths.intake2),
                /// Shooting
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, paths.intake3),
                /// Shooting
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.END)
        );
        schedule(AutoSequence
                .alongWith((new InstantCommand(() -> shooter.moveServo(.8)))));
    }

    @Override
    public void run() {
        ARC();
        updateShooter();
        super.run();
    }
}
