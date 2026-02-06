package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp
@Config
public class TurretTuner extends CommandOpMode {

    private AbsoluteAnalogEncoder encoder;
    public double realCurrPos;
    public static double setPoint;
    public static double kP, kD, kF, kI;
    public CRServoEx servo1;
    public CRServoEx servo2;
    public double lastPos;
    Shooter shooter;
    Turret turret;

    public static double pwr;

    PIDFController PID = new PIDFController(0.8,0,.005,0.015);
    @Override
    public void initialize(){
        turret = new Turret(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo1 = new CRServoEx(hardwareMap, "turret1").setCachingTolerance(.0005);
        servo2 = new CRServoEx(hardwareMap, "turret2").setCachingTolerance(.0005);
        shooter = new Shooter(hardwareMap);
        encoder = new AbsoluteAnalogEncoder(hardwareMap, "turretEncoder");
        PID.setTolerance(1);
    }

    @Override
    public void run(){
        PID.setCoefficients(new PIDFCoefficients(kP, kI, kD, 0.012));
        getRealPosition();
        if (setPoint < turret.getPos());{
            PID.setCoefficients(new PIDFCoefficients(kP, kI, kD, 0.02));
        }
        double output = PID.calculate(realCurrPos, setPoint);
        servo1.set(output);
        servo2.set(output);
        telemetry.addData("realcurrPos", turret.getPos());
        telemetry.addData("output", output);
        telemetry.addData("target", setPoint);
        telemetry.addData("Relative heading in Rad", turret.getTurretHeadingRad());
        telemetry.addData("Relative heading in deg", turret.getTurretHeadingDeg());
        telemetry.update();
        super.run();
    }

    public void getRealPosition(){
        double currentPos = MathUtils.normalizeRadians(encoder.getCurrentPosition(), false);

        if (currentPos < 0){
            if (lastPos < 0){
                realCurrPos += currentPos - lastPos;
            }
            else if (lastPos > 3){
                double delta1 = -(-Math.PI - currentPos);
                double delta2 = Math.PI - lastPos;
                realCurrPos += delta1 + delta2;
            }

        }
        else if (currentPos >= 0) {
            if (lastPos > 0) {
                realCurrPos += currentPos - lastPos;
            } else if (lastPos < -3) {
                double delta1 = -(-Math.PI - lastPos);
                double delta2 = Math.PI - currentPos;
                realCurrPos += delta1 + delta2;
            }
        }
        lastPos = currentPos;
    }
}