package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Rusty;

@Config
@TeleOp
        (name = "MainTeleOp", group = "teleop")
public class MainTeleOp extends CommandOpMode {
    Rusty rusty;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rusty = new Rusty( this);
        rusty.init();

    }

    @Override
    public void run(){
        rusty.run();
    }
}