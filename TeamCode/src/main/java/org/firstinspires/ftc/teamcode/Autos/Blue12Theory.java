package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Requirements.AutoType;
import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.Requirements.LaunchState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Blue12Theory extends CommandOpMode {

    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;
    Shooter shooter;
    Intake intake;
    Transfer transfer;

    public static LaunchState launchState;


    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        Paths = new BluePaths(AutoType.CLOSE_TWELVE_NO_TURRET, follower);

        launchState = LaunchState.SHOOTING;

        AutoSequence = new SequentialCommandGroup(
                /// Shooting preloads
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new AutoShootingCMD(intake, transfer, shooter, 1450),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake1),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore1),
                new AutoShootingCMD(intake, transfer, shooter, 1450),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake2),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore2),
                new AutoShootingCMD(intake, transfer, shooter, 1450),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake2_),
                new FollowPathCommand(follower, Paths.Intake3),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore3),
                new AutoShootingCMD(intake, transfer, shooter, 1450),
                new InstantCommand(() -> launchState = LaunchState.END),
                /// Leave
                new FollowPathCommand(follower, Paths.move)
        );

        schedule(transfer.close.alongWith(AutoSequence));
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        if (launchState == LaunchState.IDLE)
            shooter.setTo(.4);
        else if (launchState == LaunchState.SHOOTING){
            shooter.setTo(.46);
        }
        else{
            shooter.setTo(.2);
        }
    }

}
