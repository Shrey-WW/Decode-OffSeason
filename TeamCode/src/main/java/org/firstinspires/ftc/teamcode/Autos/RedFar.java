package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.AutoType;
import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.Requirements.RedPaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (group = "old")
public class RedFar extends CommandOpMode {

    /// class vars
    private Follower follower;
    RedPaths Paths;
    private static final double targetSpeed = .5;

    /// subsystems
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    Turret turret;

    /// Commands
    InstantCommand startIntake, stopIntake, reverseIntake, startFlywheel, restFlywheel, hoodUp;
    SequentialCommandGroup AutoSequence;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8.75, Math.PI / 2).mirror());
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

            Paths = new RedPaths(AutoType.FAR_NINE, follower);
        Paths.buildPaths();

        startIntake = new InstantCommand(() -> intake.Spin(1));
        stopIntake = new InstantCommand(intake::PwrOff);
        reverseIntake = new InstantCommand(() -> intake.Spin(-1));
        startFlywheel = new InstantCommand(() -> shooter.setTo(.6));
        restFlywheel = new InstantCommand(() -> shooter.setTo(.3));
        hoodUp = new InstantCommand(() -> shooter.moveServo(0));

        AutoSequence = new SequentialCommandGroup(
                /// shooting preloads
                transfer.close,
                new FollowPathCommand(follower, Paths.ShootPreloads),
                hoodUp,
                startIntake,
                new WaitCommand(3000),
                transfer.open,
                new WaitCommand(5000),
                /// spitting out balls
                new FollowPathCommand(follower, Paths.move),
                /// intaking
                new FollowPathCommand(follower, Paths.Intake1),
                transfer.close,
                startIntake,
                new FollowPathCommand(follower, Paths.Intake1_),
                /// shooting
                new FollowPathCommand(follower, Paths.goToScore1),
                transfer.open,
                new WaitCommand(5000)
//                /// intaking
//                new FollowPathCommand(follower, Paths.Intake2),
//                transfer.close,
//                startIntake,
//                new FollowPathCommand(follower, Paths.Intake2_),
//                stopIntake,
//                /// shooting
//                startFlywheel,
//                new FollowPathCommand(follower, Paths.goToScore2),
//                transfer.open,
//                new WaitCommand(5000)
        );
        register(intake, shooter, transfer, turret);
        schedule(AutoSequence);

    }

    @Override
    public void run() {
        shooter.setTo(.62);
        super.run();
        follower.update();
    }
}
