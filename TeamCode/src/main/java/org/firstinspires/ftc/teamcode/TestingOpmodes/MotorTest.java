package org.firstinspires.ftc.teamcode.TestingOpmodes;


import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.SquidMotor;

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
                new SubsystemComponent(SquidMotor.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Button my = button(() -> gamepad1.a);
        my.whenBecomesTrue(() -> SquidMotor.X.PIDchange());
    }

    @Override
    public void onUpdate(){
        SquidMotor.X.SpinTo(goal).schedule();
        telemetry.addData("target", goal);
        telemetry.addData("current pos", SquidMotor.X.getPos());
        telemetry.update();
    }

}
