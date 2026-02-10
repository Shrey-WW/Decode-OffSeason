package org.firstinspires.ftc.teamcode.SolversLib.CMDs;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Globals.Paradigms.Rusty;
import org.firstinspires.ftc.teamcode.Globals.Paradigms.ShootingCMD;
import org.firstinspires.ftc.teamcode.Globals.States.LaunchState;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Turret;

public class TeleShootingCMD extends ShootingCMD {

    public TeleShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll) {
        super(s,t,i, tt, ll);
        addRequirements(shooter, transfer, intake);
    }

    @Override
    public void initialize() {
        Rusty.launchState = LaunchState.SHOOTING;
    }

    @Override
    public void execute() {
        VELO();
    }

    @Override
    public void end(boolean interrupted) {
        intake.Spin(0);
        transfer.Spin(-.6);
        Rusty.launchState = LaunchState.IDLE;
    }

}
