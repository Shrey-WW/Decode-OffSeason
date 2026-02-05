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

@TeleOp
@Config
public class TurretTuner extends CommandOpMode {

    private AbsoluteAnalogEncoder encoder;
    public double realCurrPos;
    public static double setPoint;
    public static double kP, kD;
    public CRServoEx servo1;
    public CRServoEx servo2;
    public double lastPos;
    Shooter shooter;

    public static double pwr;

    PIDFController PID = new PIDFController(0.8,0,0,0.001);
    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo1 = new CRServoEx(hardwareMap, "turret1").setCachingTolerance(.0005);
        servo2 = new CRServoEx(hardwareMap, "turret2").setCachingTolerance(.0005);
        shooter = new Shooter(hardwareMap);
        encoder = new AbsoluteAnalogEncoder(hardwareMap, "turretEncoder");
        PID.setTolerance(1);
    }

    @Override
    public void run(){
        getRealPosition();
        double output = PID.calculate(realCurrPos, setPoint);

        servo1.set(output);
        servo2.set(output);
        telemetry.addData("realcurrPos", realCurrPos);
        telemetry.addData("output", output);
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
            if (lastPos > 0){
                realCurrPos += currentPos - lastPos;
            }
            else if(lastPos < -3){
                double delta1 = -(-Math.PI - lastPos);
                double delta2 = Math.PI - currentPos;
                realCurrPos += delta1 + delta2;
            }
        }
        lastPos = currentPos;
    }
}