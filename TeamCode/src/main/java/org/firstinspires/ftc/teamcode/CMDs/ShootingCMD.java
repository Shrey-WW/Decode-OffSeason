package org.firstinspires.ftc.teamcode.CMDs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;


public abstract class ShootingCMD extends CommandBase {

    protected InterpLUT pwr2velo = new InterpLUT();
    protected InterpLUT VELO = new InterpLUT();
    protected final Shooter shooter;
    protected final Intake intake;
    protected final Transfer transfer;
    protected Limelight3A limelight;

    protected static final double DefaultTargetVel = 980;
    protected double TargetVel = 0;

    public ShootingCMD(Shooter s, Transfer t, Intake i, Limelight3A ll){
        shooter = s;
        transfer = t;
        intake = i;
        limelight = ll;
        VELO.add(.18, .67);
        VELO.add(.2, .6);
        VELO.add(.305, .59);
        VELO.add(.45, .5075);
        VELO.add(.69, .475);
        VELO.add(1.05, .4467);
        VELO.add(1.61,.43);
        VELO.add(2.81, .424);
        VELO.add(5.5, .403);

        pwr2velo.add(.3, 700);
        pwr2velo.add(.5, 1200);
        pwr2velo.add(.7, 1700);
        pwr2velo.add(.9, 2200);

        VELO.createLUT();
        pwr2velo.createLUT();
    }

    protected void VELO(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double pwr = VELO.get(llResult.getTa());
            TargetVel = pwr2velo.get(pwr);
            shooter.setTo(pwr);
            double currentVel = shooter.getVelo();
            if (currentVel > TargetVel - 125) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(0);
                intake.Spin(.7);
            }
        }
        else{
            double currentVel = shooter.getVelo();
            shooter.setTo(.44);
            if (currentVel > DefaultTargetVel - 125) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(0);
                intake.Spin(.7);
            }
        }
    }
}
