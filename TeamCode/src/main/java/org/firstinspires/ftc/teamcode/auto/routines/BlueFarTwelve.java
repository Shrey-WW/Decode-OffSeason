package org.firstinspires.ftc.teamcode.auto.routines;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.AutoState;
import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.constants.LaunchState;

@Autonomous (group = "a. New")
public class BlueFarTwelve extends AutoBase {

        protected InterpLUT pwr2velo = new InterpLUT();
        protected double TargetVel = 0;

        @Override
        public void initialize(){
            alliance = Alliance.BLUE;
            autoType = AutoType.FAR_TWELVE_NO_TURRET;
            startingPose = new Pose(56, 8.75, Math.PI/2);

            pwr2velo.add(0, 0);
            pwr2velo.add(.3, 700);
            pwr2velo.add(.5, 1200);
            pwr2velo.add(.7, 1700);
            pwr2velo.add(.9, 2200);

            pwr2velo.createLUT();
            super.initialize();

            AutoSequence = new SequentialCommandGroup(
                    new FollowPathCommand(follower, paths.ShootPreloads),
                    new InstantCommand(() -> AutoState.launchstate = LaunchState.SHOOTING),
                    new WaitCommand(3000),
                    new InstantCommand(() -> AutoState.launchstate = LaunchState.IDLE),
                    new FollowPathCommand(follower, paths.Intake1_),
                    new FollowPathCommand(follower, paths.Intake2_),
                    new FollowPathCommand(follower, paths.Intake3),
                    new FollowPathCommand(follower, paths.fillerPath),
                    new FollowPathCommand(follower, paths.fillerPath2),
                    new FollowPathCommand(follower, paths.goToScore3),
                    new InstantCommand(() -> AutoState.launchstate = LaunchState.SHOOTING),
                    new WaitCommand(3000),
                    new InstantCommand(() -> AutoState.launchstate = LaunchState.IDLE),
                    new FollowPathCommand(follower, paths.Intake1),
                    new FollowPathCommand(follower, paths.goToScore1),
                    new InstantCommand(() -> AutoState.launchstate = LaunchState.SHOOTING),
                    new WaitCommand(3000),
                    new InstantCommand(() -> AutoState.launchstate = LaunchState.IDLE),
                    new FollowPathCommand(follower, paths.leave)
            );
            schedule(AutoSequence
                            .alongWith(new InstantCommand( () -> shooter.moveServo(.8))));
        }

        @Override
        public void run(){
            if (AutoState.launchstate == LaunchState.SHOOTING){
                shooting();
            }
            else if (AutoState.launchstate == LaunchState.IDLE){
                shooter.setVelocity(.45);
                intake.Spin(1);
                transfer.Spin(-.6);
            }
            telemetry.addData("turret pos", turret.getPosTicks());
            telemetry.addData("tx", limelight.getLatestResult().getTx());
            super.run();
        }

        public void shooting(){
            shooter.setVelocity(.62);
            TargetVel = pwr2velo.get(.62);
            double currentVel = shooter.getVelo();
            if (currentVel > TargetVel - 30){
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(-.6);
                intake.Spin(.7);
            }
        }
    }
