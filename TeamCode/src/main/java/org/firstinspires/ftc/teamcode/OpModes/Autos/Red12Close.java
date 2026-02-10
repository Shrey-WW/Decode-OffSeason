package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Globals.States.Alliance;
import org.firstinspires.ftc.teamcode.SolversLib.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.Globals.States.AutoStates;
import org.firstinspires.ftc.teamcode.Globals.States.AutoType;
import org.firstinspires.ftc.teamcode.Globals.States.LaunchState;
import org.firstinspires.ftc.teamcode.Globals.Paradigms.AutoBase;

@Autonomous (group = "a. New")
public class Red12Close extends AutoBase {
    
    @Override
    public void initialize(){
        startingPose = new Pose(19, 123, Math.toRadians(144)).mirror();
        autoType = AutoType.CLOSE_TWELVE_NO_TURRET;
        alliance = Alliance.RED;
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
                new InstantCommand(() -> AutoStates.launchstate = LaunchState.END)
        );
        schedule(AutoSequence.alongWith((new InstantCommand(() -> shooter.moveServo(.8))))
                .alongWith(new InstantCommand(() -> transfer.setPos(0))));

    }

    @Override
    public void run() {
        if (AutoState.launchstate == LaunchState.SHOOTING) {
            ARC();
        }
        super.run();
    }
}
