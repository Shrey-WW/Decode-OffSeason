package org.firstinspires.ftc.teamcode.Subsystems_Solvers;

import static org.firstinspires.ftc.teamcode.Subsystems_Solvers.Rusty2.Tx;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Config
public class Turret extends SubsystemBase {
    private final MotorEx motor;

    public static double Kp, Ki, Kd, Ks;
    public double kp, ki, kd, ks;
    private double pwr = 0;
    private double cPos = 0;
    private static final double TICKS_PER_REV = 2403.125;
    private static final double UnwindThreshold = 1400;

    // velo:  kP = .00028 Ks = 310
    //pos: kP = .0018
    public Turret(final HardwareMap hw){
        motor = new MotorEx(hw, "turret", Motor.GoBILDA.RPM_435);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(kp, 0, 0);
        motor.setFeedforwardCoefficients(ks, 0);
    }


    public void setVelocity(double input){
        motor.setVelocity(input);
    }

    public void goToPos(int input){
        motor.setTargetPosition(input);
        motor.set(2);
    }

    public void setVelocityControl(){
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(.00028, 0, 0);
        motor.setFeedforwardCoefficients(425, 0);
    }

    public void increaseFriction(){
        motor.setFeedforwardCoefficients(1000, 0);
    }

    public void setPositionControl(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionCoefficient(.0018);
    }

    public void updatePID(){
        kp = Kp;
        ki = Ki;
        kd = Kd;
        ks = Ks;
        motor.setPositionCoefficient(kp);
        motor.setFeedforwardCoefficients(ks, 0);
    }

    public double getPos(){
        return motor.getCurrentPosition();
    }

    public double getVelo(){
        return motor.getVelocity();
    }


}