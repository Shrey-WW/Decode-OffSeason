package org.firstinspires.ftc.teamcode.Tests_and_Tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Old_Turret;

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
                new SubsystemComponent(Old_Turret.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void onUpdate() {
        if (Old_Turret.X.getPIDGains().kP != Old_Turret.kP || Old_Turret.X.getPIDGains().kD != Old_Turret.Kd || Old_Turret.X.getPIDGains().kI != Old_Turret.Ki)
        {
            Old_Turret.X.velPID();
        }
        Old_Turret.X.runTo(target).schedule();
        telemetry.addData("target", target);
        telemetry.addData("current velo", Old_Turret.X.getVelo());
        telemetry.addData("current pos", Old_Turret.X.getPos());
        telemetry.update();
    }
}
