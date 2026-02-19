package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.internal.network.RobotCoreCommandList;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp (group = "tuning")
@Config
public class FlywheelTuner extends CommandOpMode{
    public static double velo;
    public static double pwr, pwr2;
    MotorEx shooter2;
    Motor shooter1;
    public static double kP = 1.5, kD, kF = 1.5;
    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter2 = new MotorEx(hardwareMap, "shooter2", Motor.GoBILDA.BARE);
        shooter2.setRunMode(Motor.RunMode.VelocityControl);
        shooter1 = new Motor(hardwareMap, "shooter1", Motor.GoBILDA.BARE);
    }

    @Override
    public void run(){
        shooter2.setVeloCoefficients(kP, 0, kD);
        shooter2.setFeedforwardCoefficients(0, kF);
        shooter2.setVelocity(velo);
        shooter1.set(shooter2.getRawPower());
        telemetry.addData("vel", shooter2.getVelocity());
        telemetry.addData("target", velo);
        telemetry.update();
        super.run();
    }
}
