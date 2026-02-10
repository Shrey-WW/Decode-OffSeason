package org.firstinspires.ftc.teamcode.SolversLib.CMDs;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals.Paradigms.ShootingCMD;
import org.firstinspires.ftc.teamcode.Globals.States.AutoStates;
import org.firstinspires.ftc.teamcode.Globals.States.LaunchState;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Turret;


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
        this(s, t, i, tt, ll, 3000);
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

        if (currentVelocity >= getTargetVel() - 50){
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
        if (cVel <= getTargetVel() - 60 && !isRecovering){
            numBallsShot++;
            isRecovering = true;
        }
    }


}
