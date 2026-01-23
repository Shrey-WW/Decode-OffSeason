package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;

@Config
@TeleOp
public class TurretTuner extends CommandOpMode {
    private CRServoGroup servoGroup;
    private Follower follower;
    private AbsoluteAnalogEncoder encoder;

    public static double setPoint;
    public static double kP, kD, kI, kF;

    SquIDFController squIDFController = new SquIDFController(0,0,0,0);
    @Override
    public void initialize(){
        encoder = new AbsoluteAnalogEncoder(hardwareMap, "turretEncoder");
        servoGroup = new CRServoGroup(
                new CRServoEx(hardwareMap, "turret1", encoder, CRServoEx.RunMode.RawPower).setCachingTolerance(.1),
                new CRServoEx(hardwareMap, "turret2").setCachingTolerance(.1).setRunMode(CRServoEx.RunMode.RawPower).setInverted(true));
    }

    @Override
    public void run(){
        squIDFController.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
        squIDFController.setSetPoint(setPoint);
        double output = squIDFController.calculate(servoGroup.getCurrentPosition());
        servoGroup.set(output);
        super.run();
    }
}