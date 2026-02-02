package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;

@Config
@TeleOp
public class TurretTuner extends CommandOpMode {
    private CRServoGroup servoGroup;
    private AbsoluteAnalogEncoder encoder;
    public static double setPoint;
    public static double kP, kD, kI, kF;
    CRServoEx servo1 = new CRServoEx(hardwareMap, "turret1").setCachingTolerance(.005);
    CRServoEx servo2 = new CRServoEx(hardwareMap, "turret2").setCachingTolerance(.005);

    public static double pwr, pwr2;

    SquIDFController squIDFController = new SquIDFController(0,0,0,0);
    @Override
    public void initialize(){
        servo2.setInverted(true);
        encoder = new AbsoluteAnalogEncoder(hardwareMap, "turretEncoder");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servoGroup = new CRServoGroup(
                new CRServoEx(hardwareMap, "turret1").setCachingTolerance(.005),
                new CRServoEx(hardwareMap, "turret2").setCachingTolerance(.005).setInverted(true));
    }

    @Override
    public void run(){
//        squIDFController.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
//        double output = squIDFController.calculate(servoGroup.getCurrentPosition(), setPoint);
//        servoGroup.set(output);
        servo1.set(pwr);
        servo2.set(pwr2);
        telemetry.addData("pos", encoder.getCurrentPosition());
        telemetry.update();
        super.run();
    }
}