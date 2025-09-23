package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@TeleOp (name = "Static FF test")
public class VelStaticTest extends NextFTCOpMode {
    public static double target;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(velPIDMotor.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void onUpdate(){
        velPIDMotor.INSTANCE.setPwr(target);
        telemetry.addData("target", target);
        telemetry.addData("current velo", velPIDMotor.INSTANCE.getVelo());
        telemetry.addData("current pos", velPIDMotor.INSTANCE.getPos());
        telemetry.update();
    }
}
