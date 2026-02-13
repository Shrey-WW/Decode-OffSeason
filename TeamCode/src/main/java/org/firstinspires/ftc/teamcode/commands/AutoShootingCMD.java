package org.firstinspires.ftc.teamcode.commands;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.AutoStates;
import org.firstinspires.ftc.teamcode.constants.LaunchState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


public class AutoShootingCMD extends ShootingCMD {

    ElapsedTime timer = new ElapsedTime();

    private final double TimeLimit;

    private final ElapsedTime ShotChecker = new ElapsedTime();


    private boolean isRecovering = true;

    public static double numBallsShot;

    public AutoShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll, double timeout) {
        super(s, t, i, tt, ll);
        TimeLimit = timeout;
        addRequirements(intake, transfer, shooter, turret);
    }

    public AutoShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll){
        this(s, t, i, tt, ll, 2000);
    }

    @Override
    public void initialize(){
        numBallsShot = 0;
        timer.reset();
        ShotChecker.reset();
        AutoStates.launchstate = LaunchState.SHOOTING;
    }

    @Override
    public void execute(){
        double currentVelocity = shooter.getVelo();

        VELO();

        if (ShotChecker.milliseconds() > 33){
            didShoot(currentVelocity);
        }

        if (currentVelocity >= getTargetVel() - 40){
            isRecovering = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        transfer.Spin(-.6);
        intake.Spin(1);
        AutoStates.launchstate = LaunchState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= TimeLimit || numBallsShot >= 3;
    }

    public void didShoot(double cVel){
        if (cVel <= getTargetVel() - 110 && !isRecovering){
            numBallsShot += 2;
            isRecovering = true;
        }
        else if (cVel <= getTargetVel() - 50 && !isRecovering){
            numBallsShot++;
            isRecovering = true;
        }
    }


}
