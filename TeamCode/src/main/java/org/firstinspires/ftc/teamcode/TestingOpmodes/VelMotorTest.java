package org.firstinspires.ftc.teamcode.TestingOpmodes;


import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.velSquidMotor;

import dev.nextftc.bindings.Button;
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
                new SubsystemComponent(velSquidMotor.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void onUpdate(){
        if(velSquidMotor.INSTANCE.getPIDCoeff().kP != velSquidMotor.INSTANCE.p
                || velSquidMotor.INSTANCE.getPIDCoeff().kD != velSquidMotor.INSTANCE.d
                || velSquidMotor.INSTANCE.getPIDCoeff().kI != velSquidMotor.INSTANCE.i)
        {velSquidMotor.INSTANCE.PIDchange();}

        velSquidMotor.INSTANCE.runTo(target).schedule();
        telemetry.addData("target", target);
        telemetry.addData("current velo", velSquidMotor.INSTANCE.getVelo());
        telemetry.addData("current pos", velSquidMotor.INSTANCE.getPos());
        telemetry.update();
    }



}
