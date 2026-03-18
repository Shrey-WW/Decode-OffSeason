package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Logan;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.paths.RedPaths;
import org.firstinspires.ftc.teamcode.constants.AutoType;

public class Red12close extends OpMode {

    Logan logan;
    Follower follower;
    private int autoState;
    ElapsedTime shotTimer;
    Paths paths;
    @Override
    public void init(){
        logan = new Logan(this);
        follower = logan.follower;
        paths = new RedPaths(AutoType.CLOSE_12, follower);
        paths.buildPaths();
        autoState = 1;
    }

    @Override
    public void start(){
        logan.start();
        shotTimer = new ElapsedTime();
    }

    @Override
    public void loop(){
        logan.runAuto();

        switch (autoState){
            case 1:
                follower.followPath(paths.shootPreloads);
                if (!follower.isBusy()) {
                    autoState = 2;
                    logan.shooter.shoot();
                    shotTimer.reset();
                }
                break;

            case 2:
                if (shotTimer.milliseconds() > 3000){
                    autoState = 3;
                    logan.shooter.idle();
                    logan.intake.Intaking();
                    logan.transfer.Reverse();
                }
                break;

            case 3:
                follower.followPath(paths.intake1);
                if (!follower.isBusy()) {
                    autoState = 4;
                    logan.intake.RestIntake();
                    shotTimer.reset();
                }
                break;

            case 4:
                if (shotTimer.milliseconds() > 3000){
                    autoState = 5;
                    logan.shooter.idle();
                    logan.intake.Intaking();
                    logan.transfer.Reverse();
                }
                break;

            case 5:
                follower.followPath(paths.intake2);
                if (!follower.isBusy()) {
                    autoState = 6;
                }
                break;

            case 6:
                follower.followPath(paths.intakeSweep1);
                if (!follower.isBusy()) {
                    autoState = 7;
                    logan.intake.RestIntake();
                }
                break;

            case 7:
                follower.followPath(paths.goToScore1);
                if (!follower.isBusy()) {
                    autoState = 8;
                    shotTimer.reset();
                    logan.shooter.shoot();
                }
                break;

            case 8:
                if (shotTimer.milliseconds() > 3000) {
                    autoState = 9;
                    logan.shooter.idle();
                    logan.intake.Intaking();
                    logan.transfer.Reverse();
                }
                break;

            case 9:
                follower.followPath(paths.intake3);
                if (!follower.isBusy()) {
                    autoState = 10;
                    shotTimer.reset();
                    logan.shooter.shoot();
                }
                break;

            case 10:
                if (shotTimer.milliseconds() > 3000) {
                    autoState = 11;
                    logan.shooter.idle();
                    logan.intake.Intaking();
                    logan.transfer.Reverse();
                }
                break;

            case 11:
                logan.shooter.end();
                logan.intake.Off();
                logan.transfer.Off();
                break;
        }
    }
}
