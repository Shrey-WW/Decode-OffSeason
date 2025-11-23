package org.firstinspires.ftc.teamcode.Subsystems_Solvers;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import java.time.Instant;

@Config
public class Flywheel extends SubsystemBase {
    private final MotorGroup ShootingMotors;
    public static double Kp, Kv;
    public double kp, kv;
    public double t;
    public Flywheel(final HardwareMap hw){
        ShootingMotors = new MotorGroup(new Motor(hw, "shooter2", Motor.GoBILDA.BARE), new Motor(hw, "shooter1", Motor.GoBILDA.BARE));
        ShootingMotors.setRunMode(Motor.RunMode.VelocityControl);
        ShootingMotors.setVeloCoefficients(5,0,0);
        ShootingMotors.setFeedforwardCoefficients(0, 1);
    }

    public InstantCommand runTo(double input){
        return new InstantCommand(() -> ShootingMotors.set(input));
    }

    public void setTo(double input){
        ShootingMotors.set(input);
    }

    public void tune(double input){
        t = input;
        ShootingMotors.set(t);
    }

    public double getVelo(){
        return ShootingMotors.getVelocity();
    }

    public void setPIDs(){
        kp = Kp;
        kv = Kv;
        ShootingMotors.setVeloCoefficients(kp, 0, 0);
        ShootingMotors.setFeedforwardCoefficients(0, kv);
    }

    public double[] getPIDs(){
        return ShootingMotors.getVeloCoefficients();
    }

    public double[] getFF(){
        return ShootingMotors.getFeedforwardCoefficients();
    }



}
