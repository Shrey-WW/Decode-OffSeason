package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.Rusty;
import org.firstinspires.ftc.teamcode.constants.LaunchState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TeleShootingCMD extends ShootingCMD {

    public TeleShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll) {
        super(s, t, i, tt, ll);
        addRequirements(shooter, transfer, intake, turret);
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
        shooter.moveServo(.8);
        intake.Spin(0);
        transfer.Spin(-.6);
        Rusty.launchState = LaunchState.IDLE;
    }

}
