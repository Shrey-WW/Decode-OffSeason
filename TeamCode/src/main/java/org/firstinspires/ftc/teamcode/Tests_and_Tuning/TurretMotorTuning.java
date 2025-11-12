package org.firstinspires.ftc.teamcode.Tests_and_Tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@TeleOp(group = "tuning")
public class TurretMotorTuning extends NextFTCOpMode {
    public static double target;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(Turret.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void onUpdate() {
        if (Turret.X.getPIDGains().kP != Turret.p || Turret.X.getPIDGains().kD != Turret.d || Turret.X.getPIDGains().kI != Turret.i)
        {
            Turret.X.velPID();
        }
        Turret.X.runTo(target).schedule();
        telemetry.addData("target", target);
        telemetry.addData("current velo", Turret.X.getVelo());
        telemetry.addData("current pos", Turret.X.getPos());
        telemetry.update();
    }
}
