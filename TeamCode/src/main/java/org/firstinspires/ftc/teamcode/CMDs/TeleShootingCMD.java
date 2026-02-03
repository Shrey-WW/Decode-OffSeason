package org.firstinspires.ftc.teamcode.CMDs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Globals.LaunchState;
import org.firstinspires.ftc.teamcode.Globals.Rusty;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

import java.lang.annotation.Target;

public class TeleShootingCMD extends CommandBase {

    private final Shooter shooter;
    private final Transfer transfer;
    private final Intake intake;
    private Limelight3A limelight;


    private InterpLUT pwr2velo = new InterpLUT();
    private InterpLUT VELO = new InterpLUT();
    private double TargetVel = 0;
    private static final double DefaultTargetVel = 980;
    public TeleShootingCMD(Shooter s, Transfer t, Intake i, Limelight3A ll, InterpLUT lut) {
        shooter = s;
        transfer = t;
        intake = i;
        limelight = ll;
        VELO = lut;
        pwr2velo.add(.3, 700);
        pwr2velo.add(.5, 1200);
        pwr2velo.add(.7, 1700);
        pwr2velo.add(.9, 2200);
        pwr2velo.createLUT();
        addRequirements(shooter, transfer, intake);
    }

    @Override
    public void initialize() {
        transfer.setPos(0);
        Rusty.launchState = LaunchState.SHOOTING;
    }

    @Override
    public void execute() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid()) {
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

    @Override
    public void end(boolean interrupted) {
        intake.Spin(0);
        transfer.Spin(0);
        transfer.setPos(.2);
        Rusty.launchState = LaunchState.IDLE;
    }

}
