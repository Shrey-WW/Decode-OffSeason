package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.RedPaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RedClose extends CommandOpMode {
    private Follower follower;
    RedPaths Paths;
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    Turret turret;
    InstantCommand startIntake, stopIntake, reverseIntake, startFlywheel, restFlywheel, hoodUp;
    SequentialCommandGroup AutoSequence;

    @Override
    public void initialize(){
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19, 123, Math.toRadians(143.5)).mirror());
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        Paths = new RedPaths(RedPaths.AutoType.CLOSE_NINE, follower);
        Paths.buildPaths();

        startIntake = new InstantCommand(() -> intake.Spin(1));
        stopIntake = new InstantCommand(intake::PwrOff);
        reverseIntake = new InstantCommand(() -> intake.Spin(-1));
        hoodUp = new InstantCommand(() -> shooter.moveServo(0));

        AutoSequence = new SequentialCommandGroup(
                /// shooting preloads
                hoodUp,
                new FollowPathCommand(follower, Paths.ShootPreloads),
                transfer.open,
                new WaitCommand(500),
                startIntake,
                new WaitCommand(5000),
                /// intaking
                transfer.close,
                new FollowPathCommand(follower, Paths.Intake1),
                startIntake,
                new FollowPathCommand(follower, Paths.Intake1_),
                /// shooting
                new FollowPathCommand(follower, Paths.goToScore1),
                transfer.open,
                startIntake,
                new WaitCommand(5000),
                /// intaking
                transfer.close,
                new FollowPathCommand(follower, Paths.Intake2),
                startIntake,
                new FollowPathCommand(follower, Paths.Intake2_),
                /// shooting
                new FollowPathCommand(follower, Paths.goToScore2),
                transfer.open,
                startIntake,
                new WaitCommand(4500),
                new FollowPathCommand(follower, Paths.fillerPath2)

        );
        register(intake, shooter, transfer, turret);
        schedule(AutoSequence);

    }

    @Override
    public void run(){
        super.run();
        shooter.setTo(.48);
        follower.update();
    }
}
