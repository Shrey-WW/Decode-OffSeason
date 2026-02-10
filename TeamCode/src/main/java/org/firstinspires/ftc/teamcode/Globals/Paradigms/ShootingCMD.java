package org.firstinspires.ftc.teamcode.Globals.Paradigms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Turret;


public abstract class ShootingCMD extends CommandBase {

    protected InterpLUT VELO = new InterpLUT();
    protected final Shooter shooter;
    protected final Intake intake;
    protected final Transfer transfer;
    protected final Turret turret;
    protected Limelight3A limelight;

    protected static final double DefaultTargetVel = 980;
    protected double TargetVel = 0;

    public ShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll){

        shooter = s;
        transfer = t;
        intake = i;
        turret = tt;
        limelight = ll;

        VELO.add(.18, .67);
        VELO.add(.2, .6);
        VELO.add(.305, .59);
        VELO.add(.45, .5075);
        VELO.add(.69, .475);
        VELO.add(1.05, .45);
        VELO.add(1.61,.44);
        VELO.add(2.81, .43);
        VELO.add(5.5, .415);
        VELO.add(7, .2);


        VELO.createLUT();
    }

    protected void VELO(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double pwr = VELO.get(llResult.getTa());
            TargetVel = getTargetVel();
            shooter.setTo(pwr);
            double currentVel = shooter.getVelo();
            if (currentVel > TargetVel - 40 && turretAligned(llResult)) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(-.5);
                intake.Spin(.7);
            }
        }
        else{
            double currentVel = shooter.getVelo();
            shooter.setTo(.44);
            if (currentVel > DefaultTargetVel - 75) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(0);
                intake.Spin(.7);
            }
        }
    }

    protected boolean turretAligned(LLResult lr){
        double Tx = lr.getTx();
        double Ta = lr.getTa();
        if (Ta < .45)
            return Math.abs(Tx) < 7;
        return Math.abs(lr.getTx()) < 6;
    }

    protected double getTargetVel(){
        LLResult lr = limelight.getLatestResult();
        if (lr != null && lr.isValid()) {
            double pwr = VELO.get(lr.getTa());
            return pwr * 2500 - 50;
        }
        return -1;
    }
}
