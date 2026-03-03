package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.AutoType;


@Autonomous
public class ElliotFarBlue extends AutoBase {


    @Override
    public void initialize(){
        SHOOTER_IDLE_VELOCITY = 1300;
        SPIN_UP_VELOCITY = 1500;
        autoType = AutoType.ELLIOT_FAR;
        startingPose = new Pose(56.000, 8.500, Math.toRadians(90));
        super.initialize();


        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.shootPreloads),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 5000),
                new FollowPathCommand(follower, paths.intake1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 5000),
                new InstantCommand(() -> intake.Spin(1)),
                new FollowPathCommand(follower, paths.intakeSweep1),
                new InstantCommand(() -> follower.setMaxPower(.6)),
                new FollowPathCommand(follower, paths.intake2),
                new InstantCommand(() -> follower.setMaxPower(1)),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 5000),
                new FollowPathCommand(follower, paths.intake3),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, 2500),
                new FollowPathCommand(follower, paths.leave)
        );
        schedule(AutoSequence.alongWith(new InstantCommand(() -> shooter.moveServo(.7))));
    }

    @Override
    public void run(){
        ARC();
        updateShooter();
        super.run();
    }
}
