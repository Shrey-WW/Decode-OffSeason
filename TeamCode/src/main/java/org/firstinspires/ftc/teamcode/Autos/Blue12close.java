package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Globals.AutoStates;
import org.firstinspires.ftc.teamcode.Globals.AutoType;
import org.firstinspires.ftc.teamcode.Globals.BluePaths;
import org.firstinspires.ftc.teamcode.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.Globals.LaunchState;


@Autonomous (group = "a. New")
public class Blue12close extends AutoBase {

    BluePaths Paths;
    @Override
    public void initialize(){
        pipelineNum = 0;
        startingPose = new Pose(19, 121, Math.toRadians(144));
        super.initialize();

        Paths = new BluePaths(AutoType.CLOSE_TWELVE_NO_TURRET, follower);
        Paths.buildPaths();

        AutoSequence = new SequentialCommandGroup(
                /// Shooting preloads
                new FollowPathCommand(follower, Paths.ShootPreloads),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake1),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore1),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake1_),
                new InstantCommand(() -> follower.setMaxPower(.5)),
                new FollowPathCommand(follower, Paths.Intake2),
                new InstantCommand(() -> follower.setMaxPower(1)),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore2),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                /// Intaking
                new FollowPathCommand(follower, Paths.Intake2_),
                new FollowPathCommand(follower, Paths.Intake3),
                /// Shooting
                new FollowPathCommand(follower, Paths.goToScore3),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new InstantCommand(() -> AutoStates.launchstate = LaunchState.END)
        );
        schedule(AutoSequence.alongWith(new InstantCommand(() -> transfer.setPos(0))));

    }

    @Override
    public void run() {
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            telemetry.addData("Tx", Tx);
            double RadianTx = -Math.toRadians(Tx);
            double targetPosition = RadianTx + turret.getPos();
            if (targetPosition >= 6 || targetPosition <= -4)
            {
                turret.pwrOff();
            }
            else{
                turret.PIDto(targetPosition);
            }
            telemetry.addData("target pos", targetPosition);
        }
        super.run();
    }
}