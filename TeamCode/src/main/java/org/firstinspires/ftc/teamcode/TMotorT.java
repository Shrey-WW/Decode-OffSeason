package org.firstinspires.ftc.teamcode;


import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@TeleOp(name = "squid Test")
public class TMotorT extends NextFTCOpMode {
    public static int goal;
    private  GraphManager graphManager = PanelsGraph.INSTANCE.getManager();
    private TelemetryManager tele = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(TMotor.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        Button my = button(() -> gamepad1.a);
//        my.whenBecomesTrue(() -> SquidMotor.INSTANCE.PIDchange());
    }

    @Override
    public void onUpdate(){
        TMotor.INSTANCE.SpinTo(goal).schedule();
        telemetry.addData("target", goal);
        telemetry.addData("current pos", TMotor.INSTANCE.getPos());
        telemetry.update();
    }

    private void UpdateSigns(){
        graphManager.addData("goal", goal);
        graphManager.addData("current pos", TMotor.INSTANCE.getPos());
        graphManager.update();
        tele.addData("goal", goal);
        tele.addData("current pos", TMotor.INSTANCE.getPos());
        tele.update();
    }
}
