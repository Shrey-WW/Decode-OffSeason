package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TurretController;

@TeleOp
@Config
public class TurretTuner extends CommandOpMode {

    private MotorEx RevEncoder;
    public static double TargetAngle;

    public static double pPos, iPos, dPos;
    public static double pVel, iVel, dVel;
    public static double kF;

    public static boolean Tuning = true;

    private static final double TICKS_PER_DEGREE = (double) 69632 /360;
    public CRServoEx servo1;
    public CRServoEx servo2;
    Shooter shooter;
    TurretController turretController;

    public static double pwr;
    @Override
    public void initialize(){
        RevEncoder = new MotorEx(hardwareMap, "shooter1");
        shooter = new Shooter(hardwareMap);
        servo1 = new CRServoEx(hardwareMap, "turret1").setCachingTolerance(.0005);
        servo2 = new CRServoEx(hardwareMap, "turret2").setCachingTolerance(.0005);
        servo1.setInverted(true);

        turretController = new TurretController(pPos, iPos, dPos, pVel, iVel, dVel, kF);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RevEncoder.stopAndResetEncoder();
    }

    @Override
    public void run(){
        double TurretVelDegrees = TicksToDegrees(RevEncoder.getVelocity());
        double TurretPosDegrees = TicksToDegrees(RevEncoder.getCurrentPosition());

        if (!Tuning) {
            servo1.set(pwr);
            servo2.set(pwr);

            telemetry.addData("current Angle", TurretPosDegrees);
            telemetry.addData("current velocity", TurretVelDegrees);
        }
        else{
            turretController.setCoefficients(pPos, iPos, dPos, pVel, iVel, dVel, kF);
            double output = turretController.calculate(TargetAngle,
                    TicksToDegrees(TurretVelDegrees),
                    TicksToDegrees(TurretPosDegrees)
            );

            servo1.set(output);
            servo2.set(output);

            telemetry.addData("output ", turretController.getOutput());
            telemetry.addLine();
            telemetry.addData("Target Angle", TargetAngle);
            telemetry.addData("current Angle", TurretPosDegrees);
            telemetry.addData("Position error", turretController.getPositionError());
            telemetry.addLine();
            telemetry.addData("Target Velocity", turretController.getTargetVelocity());
            telemetry.addData("current velocity", TurretVelDegrees);
            telemetry.addData("Velocity error", turretController.getVelocityError());
            telemetry.addLine();

        }
        telemetry.update();
        super.run();
    }

    private double TicksToDegrees(double Ticks){
        return Ticks / TICKS_PER_DEGREE;
    }

}
