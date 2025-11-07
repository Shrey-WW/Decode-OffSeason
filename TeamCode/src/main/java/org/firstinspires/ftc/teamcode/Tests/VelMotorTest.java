package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.TurretMotor;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@TeleOp(name = "Vel Motor Test")
public class VelMotorTest extends NextFTCOpMode {
    public static int target;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(TurretMotor.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TurretMotor.X.velPID();
    }

    @Override
    public void onUpdate() {
        if (TurretMotor.X.getPIDGains().kP != TurretMotor.X.p
                || TurretMotor.X.getPIDGains().kD != TurretMotor.X.d
                || TurretMotor.X.getPIDGains().kI != TurretMotor.X.i){
            TurretMotor.X.velPID();}

        TurretMotor.X.runTo(target).schedule();
        telemetry.addData("target", target);
        telemetry.addData("current velo", TurretMotor.X.getVelo());
        telemetry.addData("current pos", TurretMotor.X.getPos());
        telemetry.update();
    }
}
