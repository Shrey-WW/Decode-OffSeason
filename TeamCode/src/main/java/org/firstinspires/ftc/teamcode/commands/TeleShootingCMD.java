package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Rusty;
import org.firstinspires.ftc.teamcode.constants.LaunchState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TeleShootingCMD extends ShootingCMD {

    boolean SpinOut = false;
    ElapsedTime timer;
    public TeleShootingCMD(Shooter s, Transfer t, Intake i, Turret tt, Limelight3A ll, VoltageSensor vs) {
        super(s, t, i, tt, ll, vs);
        addRequirements(shooter, transfer, intake, turret);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        Rusty.launchState = LaunchState.SHOOTING;
    }

    @Override
    public void execute() {
        if (timer.milliseconds() > 1600 && !SpinOut) {
            transfer.Spin(-.6);
            intake.Spin(-.7);
            SpinOut = true;
            timer.reset();
        }
        else if (SpinOut && timer.milliseconds() > 1000) {
            timer.reset();
            SpinOut = false;
        }
        else { VELO(); }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.moveServo(.8);
        transfer.Spin(-.6);
        Rusty.launchState = LaunchState.IDLE;
    }

}
