package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.Globals.AutoType;
import org.firstinspires.ftc.teamcode.Globals.LaunchState;
import org.firstinspires.ftc.teamcode.Globals.RedPaths;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (group = "a. New")
public class Red12Close extends CommandOpMode {

    private Follower follower;
    RedPaths Paths;
    SequentialCommandGroup AutoSequence;
    Shooter shooter;
    Intake intake;
    Transfer transfer;

    public static LaunchState launchState;



    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20, 123, Math.toRadians(143.5)).mirror());
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        Paths = new RedPaths(AutoType.CLOSE_TWELVE_NO_TURRET, follower);
        Paths.buildPaths();
        ElapsedTime timer;
        launchState = LaunchState.SHOOTING;

        AutoSequence = new SequentialCommandGroup(
                /// Shooting preloads
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new AutoShootingCMD(intake, transfer, shooter, 1092, 3000),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake1),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore1),
                new AutoShootingCMD(intake, transfer, shooter, 1092, 3000),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake2),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore2),
                new AutoShootingCMD(intake, transfer, shooter, 1092, 3000),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake2_),
                new FollowPathCommand(follower, Paths.Intake3),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore3),
                new AutoShootingCMD(intake, transfer, shooter, 1092, 3200),
                new InstantCommand(() -> launchState = LaunchState.END),
                /// Leave
                new FollowPathCommand(follower, Paths.move)
        );

        schedule(AutoSequence.alongWith(new InstantCommand(() -> transfer.setPos(0))));
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        if (launchState == LaunchState.IDLE)
            shooter.setTo(.4);
        else if (launchState == LaunchState.SHOOTING){
            shooter.setTo(.475);
        }
        else{
            shooter.setTo(.2);
        }
    }
}
