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
@TeleOp(name = "Motor Test")
public class MotorTest extends NextFTCOpMode {
    public static int goal;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(SquidMotor.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Button my = button(() -> gamepad1.a);
        my.whenBecomesTrue(() -> SquidMotor.INSTANCE.PIDchange());
    }

    @Override
    public void onUpdate(){
        SquidMotor.INSTANCE.SpinTo(goal).schedule();
        telemetry.addData("target", goal);
        telemetry.addData("current pos", SquidMotor.INSTANCE.getPos());
        telemetry.update();
    }

}
