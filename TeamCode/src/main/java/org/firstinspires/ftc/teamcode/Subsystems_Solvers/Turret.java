package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Turret extends SubsystemBase {
    private final Motor motor;

    public static double kP, Ki, Kd, Ks;
    private double pwr = 0;
    private double cPos = 0;

    public PIDController pid = new PIDController(kP, Ki, Kd);

    public Turret(final HardwareMap hw){
        motor = new Motor(hw, "turret");
    }
    // pos: kP = .0015 Ki = 0 Kd = 0
    // velo:  kP = .000001  Ki = 0.000000000005  Kd = 10

    public void SpeedUp(){
        pwr += .04;
        motor.set(pwr);
    }

    public void SlowDown(){
        pwr -= .04;
        motor.set(pwr);
    }

    @Override
    public void periodic(){
        pid.setTolerance(5, 10);
        double output = pid.calculate(
            motor.getCurrentPosition()
        );
        motor.setVelocity(output);
    }

    public double getPos(){
        return motor.getCurrentPosition();
    }

    public double getVelo(){
        return motor.getVelocity();
    }

    public void setPID(double kp, double ki, double kd){
        pid.setPID(kp, ki, kd);
    }
}