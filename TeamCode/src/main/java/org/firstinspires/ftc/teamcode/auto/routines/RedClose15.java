package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.AutoState;
import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.constants.LaunchState;

@Autonomous(group = "Autos")
public class RedClose15 extends AutoBase {

    double TurretPosition = 0;
    @Override
    public void initialize(){
        autoType = AutoType.CLOSE_15;
        SPIN_UP_VELOCITY = 1250;
        startingPose = new Pose(19, 121, 2.4065).mirror(144);
        alliance = Alliance.RED;
        super.initialize();

        AutoSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> TurretPosition = 58),
                        new FollowPathCommand(follower, paths.shootPreloads)
                ),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 1800),
                new ParallelCommandGroup(
                        new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                        new InstantCommand(() -> TurretPosition = 90),
                        new FollowPathCommand(follower, paths.intake1, .87)
                ),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 1800),
                new FollowPathCommand(follower, paths.intake2),
                new FollowPathCommand(follower, paths.intakeSweep1),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new InstantCommand(() -> TurretPosition = 64),
                        new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                        new InstantCommand(() -> intake.Spin(.7)),
                        new FollowPathCommand(follower, paths.goToScore2)
                ),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 1800),
                new FollowPathCommand(follower, paths.intake3),
                new FollowPathCommand(follower, paths.intakeSweep2),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new InstantCommand(() -> TurretPosition = 42.1),
                        new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                        new InstantCommand(() -> intake.Spin(.7)),
                        new FollowPathCommand(follower, paths.goToScore3)
                ),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 1800),
                new InstantCommand(() -> TurretPosition = 72.1),
                new FollowPathCommand(follower, paths.intake4, .87),
                new InstantCommand(() -> intake.Spin(.9)),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 1800)
        );
        schedule(AutoSequence, new InstantCommand(() -> shooter.moveServo(.8)));
    }

    @Override
    public void run(){
        turret.TurnTo(TurretPosition);
        updateShooter();
        super.run();
    }
}
