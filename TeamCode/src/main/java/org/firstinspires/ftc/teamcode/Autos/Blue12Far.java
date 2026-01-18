package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.Requirements.AutoType;
import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
import org.firstinspires.ftc.teamcode.Requirements.LaunchState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "a. New")
public class Blue12Far extends CommandOpMode {

    private Follower follower;
    BluePaths Paths;
    SequentialCommandGroup AutoSequence;
    Intake intake;
    Shooter shooter;
    Transfer transfer;
    public static LaunchState launchState;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8.75, Math.PI/2));
        Paths = new BluePaths(AutoType.FAR_TWELVE_NO_TURRET, follower);
        Paths.buildPaths();
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);


        AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new AutoShootingCMD(intake, transfer, shooter, 1200)
                );
    }
}
