package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.AutoType;


@Autonomous
public class BlueClose12Turret extends AutoBase {

    @Override
    public void initialize() {
        autoType = AutoType.CLOSE_TWELVE;
        startingPose = new Pose(19, 121, Math.toRadians(144));
        super.initialize();

        AutoSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.ShootPreloads),
                        new InstantCommand(() -> turret.PIDto(-2.5))
                ),

                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.Intake1),
                        new WaitUntilCommand(() -> follower.atPose(new Pose(40.5, 62), 3, 3))
                                .andThen(new InstantCommand(() -> follower.setMaxPower(.6)))
                ),
                new InstantCommand(() -> follower.setMaxPower(1)),
                new FollowPathCommand(follower, paths.openGate),
                new FollowPathCommand(follower, paths.goToScore1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new FollowPathCommand(follower, paths.openGate2),
                    new WaitCommand(200),
                new FollowPathCommand(follower, paths.Intake2),
                new ParallelCommandGroup(
                        new WaitCommand(3000),
                        new InstantCommand(() -> intake.Spin(1))
                ),
                new FollowPathCommand(follower, paths.goToScore2),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new FollowPathCommand(follower, paths.Intake3),
                new FollowPathCommand(follower, paths.goToScore3),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new FollowPathCommand(follower, paths.leave)
            );


        schedule(AutoSequence);
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        Pose cPose = follower.getPose();
        telemetry.addData("x", cPose.getX());
        telemetry.addData("y", cPose.getY());
        telemetry.addData("heading", cPose.getHeading());
        telemetry.update();
    }

}
