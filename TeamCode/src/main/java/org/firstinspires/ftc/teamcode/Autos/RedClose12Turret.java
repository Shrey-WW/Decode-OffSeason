package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.Globals.AutoType;
import org.firstinspires.ftc.teamcode.Globals.LaunchState;
import org.firstinspires.ftc.teamcode.Globals.RedPaths;
import org.firstinspires.ftc.teamcode.Globals.TurretControl;


@Disabled
@Autonomous
public class RedClose12Turret extends AutoBase {
    RedPaths Paths;

    public static LaunchState launchState;
    public static TurretControl turretControl;

    @Override
    public void initialize() {
        pipelineNum = 0;
        startingPose = new Pose(19, 123, Math.toRadians(144));
        turretControl = TurretControl.TX_LOCK;
        super.initialize();
        Paths = new RedPaths(AutoType.CLOSE_TWELVE, follower);
        Paths.buildPaths();
        AutoSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Paths.ShootPreloads),
                        new AutoShootingCMD(shooter, transfer, intake, turret, limelight)
                ),
                new InstantCommand(() -> turretControl = TurretControl.NONE),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Paths.Intake1),
                        new WaitUntilCommand(() -> follower.atPose(new Pose(40.5, 62), 3, 3))
                                .andThen(new InstantCommand(() -> follower.setMaxPower(.6)))
                ),
                new InstantCommand(() -> follower.setMaxPower(1)),
                new FollowPathCommand(follower, Paths.openGate),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Paths.goToScore1),
                        new InstantCommand(() -> turretControl = TurretControl.TX_LOCK)),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new InstantCommand(() -> turretControl = TurretControl.NONE),
                new FollowPathCommand(follower, Paths.openGate2),
                    new WaitCommand(200),
                new FollowPathCommand(follower, Paths.Intake2),
                new ParallelCommandGroup(
                        new WaitCommand(3000),
                        new InstantCommand(() -> intake.Spin(1))
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Paths.goToScore2),
                        new InstantCommand(() -> turretControl = TurretControl.TX_LOCK)),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new InstantCommand(() -> turretControl = TurretControl.NONE),
                new FollowPathCommand(follower, Paths.Intake3),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Paths.goToScore3),
                        new InstantCommand(() -> turretControl = TurretControl.TX_LOCK)),
                new AutoShootingCMD(shooter, transfer, intake, turret, limelight),
                new InstantCommand(() -> turretControl = TurretControl.NONE),
                new FollowPathCommand(follower, Paths.leave)
            );


        schedule(AutoSequence);
    }

    @Override
    public void run() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
//        double vOmega = follower.getAngularVelocity();
//        double requiredTurretSpeed = -vOmega;
//
//        double feedforwardPower = requiredTurretSpeed * kA;

        if (turretControl == TurretControl.TX_LOCK){
            if (llresult != null && llresult.isValid()) {
                double Tx = llresult.getTx();
                telemetry.addData("Tx", Tx);
                double RadianTx = -Math.toRadians(Tx);
                double targetPosition = RadianTx + turret.getPos();
                telemetry.addData("target pos", targetPosition);
                double parallax = follower.getAngularVelocity();
                double HeadingFF = parallax * .05;
                turret.PIDto(targetPosition, HeadingFF);
            }
            else {
                double parallax = follower.getAngularVelocity();
                double HeadingFF = parallax * .05;
                turret.setTo(HeadingFF);
            }
        }
        else {turret.pwrOff();}
        super.run();
        follower.update();
        Pose cPose = follower.getPose();
        telemetry.addData("x", cPose.getX());
        telemetry.addData("y", cPose.getY());
        telemetry.addData("heading", cPose.getHeading());
        telemetry.update();
    }

}
