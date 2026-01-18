package org.firstinspires.ftc.teamcode.CMDs;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Autos.Blue12close;
import org.firstinspires.ftc.teamcode.Requirements.LaunchState;
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

    public AutoShootingCMD(Intake i, Transfer t, Shooter s, double TargetVelocity, double timeout) {
        intake = i;
        transfer = t;
        shooter = s;
        TargetVel = TargetVelocity;
        TimeLimit = timeout;
        addRequirements(intake, transfer, shooter);
    }

    public AutoShootingCMD(Intake i, Transfer t, Shooter s, double TargetVelocity){
        intake = i;
        transfer = t;
        shooter = s;
        TargetVel = TargetVelocity;
        TimeLimit = 3000;
        addRequirements(intake, transfer, shooter);
    }

    @Override
    public void initialize(){
        transfer.setPos(0);
        timer.reset();
        Blue12close.launchState = LaunchState.SHOOTING;
    }

    @Override
    public void execute(){
        double CurrentVel = shooter.getVelo();

        if (CurrentVel > TargetVel - 20){
            transfer.Spin(1);
            intake.Spin(1);
        }
        else
        {
            transfer.Spin(-1);
            intake.Spin(-.3);
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
        return timer.milliseconds() >= TimeLimit;
    }


}
