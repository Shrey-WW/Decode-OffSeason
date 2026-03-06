package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.AutoType;

@Autonomous(group = "Autos")
public class ElliotFarRed extends AutoBase {

    @Override
    public void initialize(){
        SHOOTER_IDLE_VELOCITY = 1400;
        SPIN_UP_VELOCITY = 1400;
        autoType = AutoType.ELLIOT_FAR;
        startingPose = new Pose(56.000, 8.500, Math.toRadians(90)).mirror(144);
        alliance = Alliance.RED;
        super.initialize();


        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.shootPreloads),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 4000),
                new FollowPathCommand(follower, paths.intake1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 4000),
                new FollowPathCommand(follower, paths.intakeSweep1),
                new FollowPathCommand(follower, paths.intake2),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 4000),
                new FollowPathCommand(follower, paths.intake3),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor, 4000),
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
