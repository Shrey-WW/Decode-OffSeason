package org.firstinspires.ftc.teamcode.CMDs;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Autos.Blue12close;
import org.firstinspires.ftc.teamcode.Globals.AutoStates;
import org.firstinspires.ftc.teamcode.Globals.LaunchState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


public class AutoShootingCMD extends ShootingCMD {

    ElapsedTime timer = new ElapsedTime();

    private final double TimeLimit;

    private final ElapsedTime ShotChecker = new ElapsedTime();

    private double lastVel;

    public static double numBallsShot;

    public AutoShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll, double timeout) {
        super(s, t, i, tt, ll);
        TimeLimit = timeout;
        addRequirements(intake, transfer, shooter);
    }

    public AutoShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll){
        this(s, t, i, tt, ll, 3000);
    }

    @Override
    public void initialize(){
        transfer.setPos(0);
        numBallsShot = 0;
        timer.reset();
        ShotChecker.reset();
        AutoStates.launchstate = LaunchState.SHOOTING;
    }

    @Override
    public void execute(){
        double currentVelocity = shooter.getVelo();

        VELO();

        if (ShotChecker.milliseconds() > 66) {
            didBallShoot(currentVelocity);
            ShotChecker.reset();
        }

    }

    @Override
    public void end(boolean interrupted) {
        transfer.Spin(-1);
        intake.Spin(1);
        transfer.setPos(.2);
        AutoStates.launchstate = LaunchState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= TimeLimit || numBallsShot >= 3;
    }

    public void didBallShoot(double cVel){
        if ((lastVel - cVel) > 50)
        {
            numBallsShot++;
        }
        lastVel = cVel;
    }


}
