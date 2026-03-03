package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


public abstract class ShootingCMD extends CommandBase {

    protected InterpLUT VELO = new InterpLUT();
    protected final Shooter shooter;
    protected final Intake intake;
    protected final Transfer transfer;
    protected final Turret turret;
    protected Limelight3A limelight;

    private static final double HEIGHT_LIMELIGHT = 16.5;
    private static final double LIMELIGHT_MOUNT_ANGLE = 12.68;
    private static final double HEIGHT_OF_APRILTAG = 29.5;


    protected static final double DefaultTargetVel = 1200;
    protected double TargetVel = 0;

    private static final double DISTANCE_FILTER_ALPHA = 0.20;
    private double filteredDistance = Double.NaN;

    public ShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll){
        shooter = s;
        transfer = t;
        intake = i;
        turret = tt;
        limelight = ll;
        VELO.add(0, 800);
        VELO.add(24.4, 1050);
        VELO.add(41, 1100);
        VELO.add(66, 1260);
        VELO.add(76, 1280);
        VELO.add(90, 1640);
        VELO.add(100, 1800);
        VELO.add(120, 1860);
        VELO.createLUT();
    }

    protected void VELO(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double velo = VELO.get(getDistanceFromTag(llResult));
            TargetVel = velo;
            shooter.setVelocity(velo);
            double currentVel = shooter.getVelo();
            if (currentVel > velo - getRecoveryOffset(velo)) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(.4);
                intake.Spin(.5);
            }
        }
        else{
            TargetVel = DefaultTargetVel;
            double currentVel = shooter.getVelo();
            shooter.setVelocity(DefaultTargetVel);
            if (currentVel > DefaultTargetVel - 140) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(.4);
                intake.Spin(.5);
            }
        }
    }

    private double getDistanceFromTag(@NonNull LLResult lr){
        double raw = (HEIGHT_OF_APRILTAG - HEIGHT_LIMELIGHT) /
                (Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + lr.getTy())) * Math.cos(Math.toRadians(lr.getTx())));
        if (Double.isNaN(filteredDistance)) {
            filteredDistance = raw;
        } else {
            filteredDistance = DISTANCE_FILTER_ALPHA * raw + .8 * filteredDistance;
        }
        if (filteredDistance >= 85)
        {
            shooter.moveServo(.7);
        }
        return filteredDistance;
    }

    private double getRecoveryOffset(double tVel){
        return -0.2 * tVel + 380;
    }
}
