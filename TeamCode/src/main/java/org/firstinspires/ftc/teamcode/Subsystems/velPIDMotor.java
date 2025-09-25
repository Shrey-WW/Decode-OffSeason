package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class velPIDMotor implements Subsystem {
    public static final velPIDMotor INSTANCE = new velPIDMotor();
    private velPIDMotor() { }

    private MotorEx motor = new MotorEx("bl");
    public static double p,i,d;

    private PIDCoefficients PIDCoeff = new PIDCoefficients(p,i,d);

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(PIDCoeff)
            .basicFF(0,0,.091)
            .build();

    @Override
    public void periodic(){
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

    public Command runTo(int goal){
        return new RunToVelocity(controlSystem, goal).requires(this);
    }

    public void setPwr(double t){
        motor.setPower(t);
    }

    public double getPos(){
        return motor.getCurrentPosition();
    }

    public PIDCoefficients getPIDCoeff(){
        return PIDCoeff;
    }

    public void PIDchange(){
        PIDCoeff = new PIDCoefficients(p,i,d);
        controlSystem = ControlSystem.builder()
                .velPid(PIDCoeff)
                .basicFF(0,0,.091)
                .build();
    }

    public double getVelo(){
        return motor.getVelocity();
    }

    public Command FollowCam(double bearing){
        return new RunToVelocity(controlSystem, motor.getCurrentPosition() - bearing * 4.7);
    }

}