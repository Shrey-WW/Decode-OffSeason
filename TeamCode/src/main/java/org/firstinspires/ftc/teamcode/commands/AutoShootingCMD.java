package org.firstinspires.ftc.teamcode.commands;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.AutoState;
import org.firstinspires.ftc.teamcode.constants.LaunchState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


public class AutoShootingCMD extends ShootingCMD {

    ElapsedTime timer = new ElapsedTime();

    private final double TimeLimit;



    private final ElapsedTime ShotChecker = new ElapsedTime();
    public static double numBallsShot;
    private double localPeakVelocity = 0.0;
    private double lastShotTime;

    public AutoShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll, double timeout) {
        super(s, t, i, tt, ll);
        TimeLimit = timeout;
        addRequirements(intake, transfer, shooter, turret);
    }

    public AutoShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll){
        this(s, t, i, tt, ll, 2500);
    }

    @Override
    public void initialize(){
        numBallsShot = 0;
        timer.reset();
        ShotChecker.reset();
        AutoState.launchstate = LaunchState.SHOOTING;
    }

    @Override
    public void execute(){

        double cVel = shooter.getVelo();

        VELO();

        didBallShoot(cVel);
    }

    @Override
    public void end(boolean interrupted) {
        transfer.Spin(-.6);
        intake.Spin(1);
        AutoState.launchstate = LaunchState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= TimeLimit || numBallsShot >= 3;
    }

    private void didBallShoot(double cVel){
        if (cVel > 200) {
            if (cVel > localPeakVelocity) {
                localPeakVelocity = cVel;
            }

            if ((localPeakVelocity - cVel) > 50) {

                if (timer.milliseconds() - lastShotTime > 30) {
                    numBallsShot++;
                    lastShotTime = timer.milliseconds();
                    localPeakVelocity = cVel;
                }
            }
        }
        else {
            localPeakVelocity = 0.0;
        }
    }


}
