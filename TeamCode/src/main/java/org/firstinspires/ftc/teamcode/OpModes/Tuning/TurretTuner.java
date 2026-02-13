package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Turret;

@TeleOp
@Config
public class TurretTuner extends CommandOpMode {

    private MotorEx RevBoreEncoder;
    public double realCurrPos;
    public static double setPoint;
    public static double kP, kD, kF, kI;
    private static final double TICKS_PER_DEGREE = 191;
    private static final double TICKS_PER_RADIANS = 10944.3;
    public CRServoEx servo1;
    public CRServoEx servo2;
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
        servo1.setInverted(true);
        shooter = new Shooter(hardwareMap);
        RevBoreEncoder = new MotorEx(hardwareMap, "shooter1");
    }

    @Override
    public void run(){
        if (gamepad1.a){
            realCurrPos = 0;
        }
        double output = PID.calculate(realCurrPos, setPoint);
        servo1.set(pwr);
        servo2.set(pwr);
        telemetry.addData("realcurrPos", RevBoreEncoder.getCurrentPosition());
        telemetry.addData("output", output);
        telemetry.addData("target", setPoint);
        telemetry.addData("Relative heading in Rad", turret.getTurretHeadingRad());
        telemetry.addData("Relative heading in deg", turret.getTurretHeadingDeg());
        telemetry.update();
        super.run();
    }

}