package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;


@TeleOp (group = "tuning")
@Config
public class FlywheelTuner extends CommandOpMode{
    public static double velo;
    public static double pwr, pwr2;
    MotorGroup ShootingMotors;
    MotorEx shooter2;
    Motor shooter1;
    public static double kP = 1.5, kD, kF = 1.5, kI = 0;
    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter2 = new MotorEx(hardwareMap, "shooter2", Motor.GoBILDA.BARE);
        shooter2.setRunMode(Motor.RunMode.VelocityControl);
        shooter1 = new Motor(hardwareMap, "shooter1", Motor.GoBILDA.BARE);
        ShootingMotors = new MotorGroup(shooter2, shooter1);
    }

    @Override
    public void run(){
        ShootingMotors.setVeloCoefficients(kP, kI, kD);
        ShootingMotors.setFeedforwardCoefficients(0, kF);
        ShootingMotors.set((velo+50)/2500);
        telemetry.addData("vel", shooter2.getVelocity());
        telemetry.addData("target", velo);
        telemetry.update();
        super.run();
    }
}
