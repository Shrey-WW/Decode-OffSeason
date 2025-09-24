package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class velPIDMotor implements Subsystem {
    public static final velPIDMotor INSTANCE = new velPIDMotor();
    private velPIDMotor() { }

    private MotorEx motor = new MotorEx("bl");
// p: .01 i: .016
    public static double p,i,d;

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(.008, 0, 0)
            .basicFF(0,0,.12)
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

    public void PIDchange(){
        controlSystem = ControlSystem.builder()
                .velPid(p,i,d)
                .basicFF(0,0,.12)
                .build();
    }

    public double getVelo(){
        return motor.getVelocity();
    }

    public double getGoal(double bearing){
        return motor.getCurrentPosition() - bearing * 4.7;
    }

    public Command FollowCam(double bearing){
        return new RunToPosition(controlSystem, motor.getCurrentPosition() - bearing * 4.7);
    }

}