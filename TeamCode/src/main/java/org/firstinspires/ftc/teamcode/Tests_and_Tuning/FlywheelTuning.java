package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@TeleOp (group = "tuning")
public class FlywheelTuning extends NextFTCOpMode {

    public static double target;
    @Override
    public void onInit(){
        addComponents(
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(Shooter.X)
        );
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void onUpdate(){
        if (Shooter.X.getPIDCoefficients().kP != Shooter.kP || Shooter.kV != Shooter.X.kv || Shooter.Ks != Shooter.X.kv || Shooter.X.getPIDCoefficients().kD != Shooter.kD){
            Shooter.X.updatePID();
        }
        Shooter.X.setVelocity(target).schedule();
        telemetry.addData("Velocity", Shooter.X.getVelo());
        telemetry.addData("target", target);
        telemetry.addData("Position", Shooter.X.getPos());
        telemetry.update();
    }
}
