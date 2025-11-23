package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Flywheel;

@Config
@TeleOp (group = "tuning")
public class FlywheelTuning extends CommandOpMode {

    public static double target;
    Flywheel flywheel;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void run(){
        double[] Gains = flywheel.getPIDs();
        double[] ff = flywheel.getFF();
        if (Gains[0] != Flywheel.Kp || ff[1] != Flywheel.Kv) {
            flywheel.setPIDs();
            flywheel.tune(target);
        }

        if (flywheel.t != target)
            flywheel.tune(target);

        telemetry.addData("Velocity", flywheel.getVelo());
        telemetry.addData("target", target);
        telemetry.update();
        CommandScheduler.getInstance().run();
    }
}
