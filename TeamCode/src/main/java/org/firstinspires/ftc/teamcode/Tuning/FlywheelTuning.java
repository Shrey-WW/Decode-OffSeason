package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@Config
@TeleOp (group = "tuning")
public class FlywheelTuning extends CommandOpMode {

    public static double target;
    Shooter shooter;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void run(){
            double[] Gains = shooter.getPIDs();
            double[] ff = shooter.getFF();
            if (Gains[0] != shooter.Kp || ff[1] != shooter.Kv) {
                shooter.setPIDs();
                shooter.tune(target);
            }

            if (shooter.t != target)
                shooter.tune(target);

        shooter.setTo(target);
        telemetry.addData("Velocity", shooter.getVelo());
        telemetry.addData("target", target);
        telemetry.update();
        CommandScheduler.getInstance().run();
    }
}
