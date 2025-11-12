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
        PIDCoefficients gains = Shooter.X.getPIDCoefficients();
        if (gains.kP != Shooter.p || gains.kD != Shooter.d || gains.kI != Shooter.i){
            Shooter.X.updatePID();
        }
        Shooter.X.setVelocity(target);
        telemetry.addData("Velocity", Shooter.X.getVelo());
        telemetry.addData("Position", Shooter.X.getPos());
    }
}
