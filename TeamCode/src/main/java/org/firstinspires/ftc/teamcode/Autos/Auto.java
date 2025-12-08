package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Auto extends CommandOpMode{
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private Follower follower;
    private Paths paths;

    @Override
    public void initialize(){
        super.reset();
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(Paths.AutoType.LINEAR_NINE, follower);
        follower.setStartingPose(paths.startPose);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        register(intake, transfer, shooter, turret);
        paths.buildPaths();
        SequentialCommandGroup AutoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.Intake1),
                new FollowPathCommand(follower, paths.goToScore1),
                new FollowPathCommand(follower, paths.Intake2),
                new FollowPathCommand(follower, paths.Intake2_),
                new FollowPathCommand(follower, paths.goToScore2, true)
        );
        schedule(AutoSequence);
    }

    @Override
    public void run(){
        super.run();
    }
}
