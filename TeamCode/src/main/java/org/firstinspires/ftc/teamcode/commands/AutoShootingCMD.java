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

    private Double overrideVelocity = Double.NaN;

    private final ElapsedTime ShotChecker = new ElapsedTime();

    private static final double ARBITRARY_CONSTANT = 35000;
    private boolean isRecovering = true, doubleShot = false;

    public static double numBallsShot;

    public AutoShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll, double timeout) {
        super(s, t, i, tt, ll);
        TimeLimit = timeout;
        addRequirements(intake, transfer, shooter, turret);
    }

    public AutoShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll){
        this(s, t, i, tt, ll, 2500);
    }

    public AutoShootingCMD(double velocity, Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll){
        this(s, t, i, tt, ll, 2500);
        overrideVelocity = velocity;
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

        double currentVelocity = shooter.getVelo();

        if (!overrideVelocity.isNaN()) {
            TargetVel = overrideVelocity;
            double currentVel = shooter.getVelo();
            shooter.setVelocity(TargetVel);
            if (currentVel > TargetVel - ARBITRARY_CONSTANT/TargetVel) {
                transfer.Spin(1);
                intake.Spin(1);
            }
            else{
                transfer.Spin(.4);
                intake.Spin(0);
            }
        }
        else {
            VELO();
        }

        if (ShotChecker.milliseconds() >= 30) {
            didShoot(currentVelocity);
            ShotChecker.reset();
        }

        if (currentVelocity >= getTargetVel() - 30){
            isRecovering = false;
            doubleShot = false;
        }
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

    public void didShoot(double cVel){
        if (cVel <= getTargetVel() - 60 && !isRecovering && !doubleShot){
            numBallsShot++;
            isRecovering = true;
        }
        if (cVel <= getTargetVel() - 70 && !isRecovering && !doubleShot) {
            numBallsShot++;
            doubleShot = true;
        }
    }


}
