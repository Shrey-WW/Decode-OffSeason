package org.firstinspires.ftc.teamcode.commands;

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

    protected static final double DefaultTargetVel = 1100;
    protected double TargetVel = 0;

    public ShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll){

        shooter = s;
        transfer = t;
        intake = i;
        turret = tt;
        limelight = ll;
        VELO.add(0, 800);
        VELO.add(24, 1000);
        VELO.add(50, 1100);
        VELO.add(67, 1220);
        VELO.add(90, 1390);
        VELO.add(110, 1480);
        VELO.add(150, 1860);
        VELO.createLUT();
    }

    protected void VELO(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double velo = VELO.get(getDistanceFromTag(llResult));
            shooter.setVelocity(velo);
            double currentVel = shooter.getVelo();
            if (currentVel > velo - 100) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(-.7);
                intake.Spin(.6);
            }
        }
        else{
            double currentVel = shooter.getVelo();
            shooter.setVelocity(800);
            if (currentVel > 800 - 100) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(-7);
                intake.Spin(6);
            }
        }
    }

    protected boolean turretAligned(LLResult lr){
        double Tx = lr.getTx();
        double Ta = lr.getTa();
        if (Ta < .45)
            return Math.abs(Tx) < 9;
        return Math.abs(Tx) < 6;
    }

    private double getDistanceFromTag(LLResult lr){
        return (HEIGHT_OF_APRILTAG - HEIGHT_LIMELIGHT) / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + lr.getTy()));
    }

    protected double getTargetVel(){
        return TargetVel;
    }
}
