package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.base.AutoBase;
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
                new FollowPathCommand(follower, paths.ShootPreloads),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, paths.Intake1),
                new FollowPathCommand(follower, paths.openGate),
                /// Shooting
                new FollowPathCommand(follower, paths.goToScore1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, paths.Intake1_),
                new InstantCommand(() -> follower.setMaxPower(.6)),
                new FollowPathCommand(follower, paths.Intake2),
                new InstantCommand(() -> follower.setMaxPower(1)),
                /// Shooting
                new FollowPathCommand(follower, paths.goToScore2),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, paths.Intake2_),
                new InstantCommand(() -> follower.setMaxPower(.6)),
                new FollowPathCommand(follower, paths.Intake3),
                new InstantCommand(() -> follower.setMaxPower(1)),
                /// Shooting
                new FollowPathCommand(follower, paths.goToScore3),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.END)
        );
        schedule(AutoSequence
                .alongWith((new InstantCommand(() -> shooter.moveServo(.8)))));
    }

    @Override
    public void run() {
        if (AutoState.launchstate == LaunchState.SHOOTING) {
            ARC();
        }
        telemetry.addData("numballshot", AutoShootingCMD.numBallsShot);
        telemetry.addData("turret pos", turret.getPosTicks());
        telemetry.addData("tx", limelight.getLatestResult().getTx());
        super.run();
    }
}
