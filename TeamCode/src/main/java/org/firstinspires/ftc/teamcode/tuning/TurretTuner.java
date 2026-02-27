package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.TurretController;

/**
 * Tuning modes
 *   0 - Manual:      direct power mode                  : pwr
 *   1 - Vel Only:    inner velocity loop                : TargetVelocity
 *   2 - Full:        full cascade (position → velocity) : TargetAngle
 */
@TeleOp(group = "tuning")
@Config
public class TurretTuner extends CommandOpMode {

    // 0 = Manual | 1 = Vel Only | 2 = Full Cascade
    public static int Mode = 0;

    // Manual
    public static double pwr = 0;

    // Velocity loop — used in modes 1 and 2
    public static double kF = .004, pVel = .0015, iVel = 0.000000005, dVel = .00001;

    // Position loop — used in mode 2 only
    public static double pPos = 7.5, iPos = .000004, dPos = 0;

    // Targets
    public static double TargetVelocity = 0;  // ONLY VEL MODE
    public static double TargetAngle    = 0;  // FULL CASCADE

    private static final double TICKS_PER_DEGREE = (double) 69632 / 360;
    private static final String[] MODE_NAMES = {"Manual", "Vel Only", "Full Cascade"};

    private MotorEx RevEncoder;
    private CRServoEx servo1, servo2;
    private TurretController turretController;

    @Override
    public void initialize() {
        RevEncoder = new MotorEx(hardwareMap, "shooter1");
        servo1 = new CRServoEx(hardwareMap, "turret1").setCachingTolerance(.0005);
        servo2 = new CRServoEx(hardwareMap, "turret2").setCachingTolerance(.0005);
        servo1.setInverted(true);
        servo2.setInverted(true);
        RevEncoder.stopAndResetEncoder();

        turretController = new TurretController(pPos, iPos, dPos, pVel, iVel, dVel, kF);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        double posDeg = ticksToDegrees(RevEncoder.getCurrentPosition());
        double velDeg = ticksToDegrees(RevEncoder.getVelocity());

        // Always-visible state
        telemetry.addData("Mode",          MODE_NAMES[Mode]);
        telemetry.addData("Position (deg)", posDeg);
        telemetry.addData("Velocity (deg/s)", velDeg);
        telemetry.addLine();

//        if (Math.abs(posDeg) >= 90) {
//            double p = Math.signum(posDeg) * -.2;
//            servo1.set(p);
//            servo2.set(p);
//            pwr = 0;
//            TargetVelocity = 0;
//            TargetAngle = 86.67;
//        }
//        else {
            switch (Mode) {
                case 0: // Manual
                    servo1.set(pwr);
                    servo2.set(pwr);
                    telemetry.addData("Power", pwr);
                    break;

                case 1: // Velocity Only
                    turretController.setVelCoefficients(pVel, iVel, dVel);
                    turretController.setFeedForward(kF);
                    double velOutput = turretController.calculateVelocityOnly(TargetVelocity, velDeg);
                    servo1.set(velOutput);
                    servo2.set(velOutput);
                    telemetry.addData("Target Velocity (deg/s)", TargetVelocity);
                    telemetry.addData("Velocity Error", turretController.getVelocityError());
                    telemetry.addData("Output", velOutput);
                    break;

                case 2: // Full Cascade
                    turretController.setCoefficients(pPos, iPos, dPos, pVel, iVel, dVel, kF);
                    double fullOutput = turretController.calculate(TargetAngle, posDeg, velDeg);
                    servo1.set(fullOutput);
                    servo2.set(fullOutput);
                    telemetry.addData("Target Angle (deg)", TargetAngle);
                    telemetry.addData("Position Error", turretController.getPositionError());
                    telemetry.addLine();
                    telemetry.addData("Target Velocity (deg/s)", turretController.getTargetVelocity());
                    telemetry.addData("Velocity Error", turretController.getVelocityError());
                    telemetry.addData("Output", fullOutput);
                    break;
            }
//        }

        telemetry.update();
        super.run();
    }

    private double ticksToDegrees(double ticks) {
        return ticks / TICKS_PER_DEGREE;
    }
}