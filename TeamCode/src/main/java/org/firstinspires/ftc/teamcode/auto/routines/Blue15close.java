package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.TrapezoidProfileCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.AutoState;
import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.constants.LaunchState;

@Autonomous (group = "Autos")
public class Blue15close extends AutoBase {
    private double TurretPosition = 0;
    @Override
    public void initialize(){
        autoType = AutoType.CLOSE_15;
        SPIN_UP_VELOCITY = 1250;
        startingPose = new Pose(19, 121, 2.4065);
        super.initialize();

        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.shootPreloads),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                new WaitCommand(750),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 2700),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                new FollowPathCommand(follower, paths.intake1, .8),
                new WaitCommand(750),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 2700),
                new FollowPathCommand(follower, paths.intake2),
                new FollowPathCommand(follower, paths.intakeSweep1),
                new WaitCommand(1250),
                new InstantCommand(() -> AutoState.launchstate = LaunchState.SPIN_UP),
                new InstantCommand(() -> intake.Spin(.7)),
                new FollowPathCommand(follower, paths.goToScore2),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 2700),
                new FollowPathCommand(follower, paths.intake3, .65),
                new InstantCommand(() -> intake.Spin(.9)),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 2700)
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
