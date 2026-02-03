package org.firstinspires.ftc.teamcode.CMDs;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Autos.Blue12close;
import org.firstinspires.ftc.teamcode.Globals.LaunchState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;


public class AutoShootingCMD extends CommandBase {
    Intake intake;
    Transfer transfer;
    Shooter shooter;

    ElapsedTime timer = new ElapsedTime();

    private final double TargetVel;

    private final double TimeLimit;

    private final ElapsedTime ShotChecker = new ElapsedTime();

    private double lastVel;

    public static double numBallsShot;

    public AutoShootingCMD(Intake i, Transfer t, Shooter s, double TargetVelocity, double timeout) {
        intake = i;
        transfer = t;
        shooter = s;
        TargetVel = TargetVelocity;
        TimeLimit = timeout;
        addRequirements(intake, transfer, shooter);
    }

    public AutoShootingCMD(Intake i, Transfer t, Shooter s, double TargetVelocity){
        this(i,t,s,TargetVelocity,3000);
    }

    @Override
    public void initialize(){
        transfer.setPos(0);
        numBallsShot = 0;
        timer.reset();
        ShotChecker.reset();
        Blue12close.launchState = LaunchState.SHOOTING;
    }

    @Override
    public void execute(){
        double currentVelocity = shooter.getVelo();
        if (currentVelocity > TargetVel - 125){
            transfer.Spin(1);
            intake.Spin(1);
        }
        else
        {
            transfer.Spin(0);
            intake.Spin(.5);
        }

        if (ShotChecker.milliseconds() > 66) {
            detectShot(currentVelocity);
            ShotChecker.reset();
        }

    }

    @Override
    public void end(boolean interrupted) {
        transfer.Spin(-1);
        intake.Spin(1);
        transfer.setPos(.2);
        Blue12close.launchState = LaunchState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= TimeLimit || numBallsShot >= 3;
    }

    public void detectShot(double cVel){
        if ((lastVel - cVel) > 50)
        {
            numBallsShot++;
        }
        lastVel = cVel;
    }


}
