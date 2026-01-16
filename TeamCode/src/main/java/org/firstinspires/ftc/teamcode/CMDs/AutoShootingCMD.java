package org.firstinspires.ftc.teamcode.CMDs;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Autos.Blue12Theory;
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

    private static final double TimeLimit = 2300;

    public AutoShootingCMD(Intake i, Transfer t, Shooter s, double TargetVelocity) {
        intake = i;
        transfer = t;
        shooter = s;
        TargetVel = TargetVelocity;
        addRequirements(intake, transfer, shooter);
    }

    @Override
    public void initialize(){
        transfer.setPos(0);
        timer.reset();
        Blue12Theory.launchState = LaunchState.SHOOTING;
    }

    @Override
    public void execute(){
        double CurrentVel = shooter.getVelo();

        if (CurrentVel > TargetVel - 50){
            transfer.Spin(1);
            intake.Spin(1);
        }
        else
        {
            transfer.Spin(0);
            intake.Spin(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        transfer.Spin(0);
        intake.Spin(1);
        transfer.setPos(.2);
        Blue12Theory.launchState = LaunchState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= TimeLimit;
    }


}
