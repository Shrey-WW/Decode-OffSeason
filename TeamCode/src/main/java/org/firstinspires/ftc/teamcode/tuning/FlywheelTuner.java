package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

/**
 * Tuning modes:
 *   0 - Manual:    direct power     : pwr
 *   1 - Velocity:  velocity control : velo
 */
@TeleOp(group = "tuning")
@Config
public class FlywheelTuner extends CommandOpMode {

    // 0 = Manual | 1 = Velocity
    public static int Mode = 0;

    // Manual
    public static double pwr = 0;

    // Velocity
    public static double velo = 0;
    public static double kP = 1.5, kI = 0, kD = 0, kF = 1.5;

    private static final String[] MODE_NAMES = {"Manual", "Velocity"};

    private MotorGroup ShootingMotors;
    private MotorEx shooter2;
    private Motor shooter1;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter2 = new MotorEx(hardwareMap, "shooter2", Motor.GoBILDA.BARE);
        shooter1 = new Motor(hardwareMap, "shooter1", Motor.GoBILDA.BARE);
        ShootingMotors = new MotorGroup(shooter2, shooter1);
    }

    @Override
    public void run() {
        telemetry.addData("Mode",     MODE_NAMES[Mode]);
        telemetry.addData("Velocity", shooter2.getVelocity());
        telemetry.addLine();

        switch (Mode) {
            case 0: // Manual
                ShootingMotors.set(pwr);
                telemetry.addData("Power", pwr);
                break;

            case 1: // Velocity
                shooter2.setRunMode(Motor.RunMode.VelocityControl);
                ShootingMotors.setVeloCoefficients(kP, kI, kD);
                ShootingMotors.setFeedforwardCoefficients(0, kF);
                ShootingMotors.set((velo + 50) / 2500);
                telemetry.addData("Target Velocity", velo);
                telemetry.addData("Error",           velo - shooter2.getVelocity());
                break;
        }

        telemetry.update();
        super.run();
    }
}