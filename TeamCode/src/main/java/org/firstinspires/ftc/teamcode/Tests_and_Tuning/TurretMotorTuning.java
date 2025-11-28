package org.firstinspires.ftc.teamcode.Tests_and_Tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Turret;


@Config
@TeleOp(group = "tuning")
public class TurretMotorTuning extends CommandOpMode {
    public static double target, posTarget;
    Turret turret;

    @Override
    public void initialize() {
        turret = new Turret(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        turret.setPositionControl();
    }

    @Override
    public void run() {
//            if (Turret.Kp != turret.kp || Turret.Ki != turret.ki || Turret.Kd != turret.kd || Turret.Ks != turret.ks)
//                turret.updatePID();

        turret.goToPos((int)target);
        telemetry.addData("target", target);
        telemetry.addData("current velo", turret.getVelo());
        telemetry.addData("current pos", turret.getPos());
        telemetry.addData( "target pos", posTarget);
        telemetry.update();
    }
}
