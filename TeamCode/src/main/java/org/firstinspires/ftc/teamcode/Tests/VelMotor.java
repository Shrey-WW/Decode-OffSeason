package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp
@Config
public class VelMotor extends CommandOpMode{
    public static double velo;
    MotorEx motor;
    public static double kP, kV;
    @Override
    public void initialize(){
        motor = new MotorEx(hardwareMap, "shooter2", Motor.GoBILDA.BARE);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(2,0,0);
        motor.setFeedforwardCoefficients(0,1);
    }

    @Override
    public void run(){
        motor.set(velo);
        motor.setVeloCoefficients(kP,0,0);
        motor.setFeedforwardCoefficients(0,kV);
        super.run();
    }
}
