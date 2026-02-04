package org.firstinspires.ftc.teamcode.CMDs;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Globals.LaunchState;
import org.firstinspires.ftc.teamcode.Globals.Rusty;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

public class TeleShootingCMD extends ShootingCMD {

    public TeleShootingCMD(Shooter s, Transfer t, Intake i, Limelight3A ll) {
        super(s,t,i,ll);
        addRequirements(shooter, transfer, intake);
    }

    @Override
    public void initialize() {
        transfer.setPos(0);
        Rusty.launchState = LaunchState.SHOOTING;
    }

    @Override
    public void execute() {
        VELO();
    }

    @Override
    public void end(boolean interrupted) {
        intake.Spin(0);
        transfer.Spin(0);
        transfer.setPos(.2);
        Rusty.launchState = LaunchState.IDLE;
    }

}
