package org.firstinspires.ftc.teamcode.TestingOpmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.velSquidMotor;

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
                new SubsystemComponent(velSquidMotor.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void onUpdate(){
        if(velSquidMotor.X.getPIDGains().kP != velSquidMotor.X.p
                || velSquidMotor.X.getPIDGains().kD != velSquidMotor.X.d
                || velSquidMotor.X.getPIDGains().kI != velSquidMotor.X.i)
        {velSquidMotor.X.velPID();}

        velSquidMotor.X.runTo(target).schedule();
        telemetry.addData("target", target);
        telemetry.addData("current velo", velSquidMotor.X.getVelo());
        telemetry.addData("current pos", velSquidMotor.X.getPos());
        telemetry.update();
    }



}
