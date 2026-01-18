package org.firstinspires.ftc.teamcode.CMDs;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Requirements.LaunchState;
import org.firstinspires.ftc.teamcode.Requirements.Rusty;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

public class TeleShootingCMD extends CommandBase {

    private final Shooter shooter;
    private final Transfer transfer;
    private final Intake intake;

    private final double TargetVel;

    public TeleShootingCMD(Shooter s, Transfer t, Intake i, double TargetVelocity) {
        shooter = s;
        transfer = t;
        intake = i;
        TargetVel = TargetVelocity;
        addRequirements(shooter, transfer, intake);
    }

    @Override
    public void initialize() {
        transfer.setPos(0);
        Rusty.launchState = LaunchState.SHOOTING;
    }

    @Override
    public void execute() {
        double currentVel = shooter.getVelo();

        if (currentVel > TargetVel - 20) {
            transfer.Spin(1);
            intake.Spin(1);
        }
        else{
            transfer.Spin(-1);
            intake.Spin(-.3);
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
