package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

@TeleOp (group = "tests")
@Config
public class VelMotor extends CommandOpMode{
    public static double velo;
    MotorGroup ShootingMotors;
    public static double kP, kV;
    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ShootingMotors = new MotorGroup(new Motor(hardwareMap, "shooter2", Motor.GoBILDA.BARE), new Motor(hardwareMap, "shooter1", Motor.GoBILDA.BARE));
        ShootingMotors.setRunMode(Motor.RunMode.VelocityControl);
        ShootingMotors.setVeloCoefficients(5,0,0);
        ShootingMotors.setFeedforwardCoefficients(0, 1);
        ShootingMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void run(){
        ShootingMotors.set(velo);
        ShootingMotors.setVeloCoefficients(kP,0,0);
        ShootingMotors.setFeedforwardCoefficients(0,kV);
        telemetry.addData("vel", ShootingMotors.getVelocity());
        telemetry.addData("target", velo);
        telemetry.update();
        super.run();
    }
}
