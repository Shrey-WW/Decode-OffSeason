package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class SquidMotor implements Subsystem {
    public static final SquidMotor INSTANCE = new SquidMotor();
    private SquidMotor() { }

    //.0.00695 , 0.0000000000005, 0.0000005
    private MotorEx motor = new MotorEx("bl");
    public static double p = .00573,i = .000000000001 ,d = 0.00000033;

    private ControlSystem controlSystem = ControlSystem.builder()
            .posSquid(.00573, .000000000001, 0.00000033)
            .build();

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

    public void PIDchange(){
        controlSystem = ControlSystem.builder()
                .posSquid(p, i, d)
                .build();
    }
    public Command SpinTo(int goal){
        return new RunToPosition(controlSystem, goal).requires(this);
    }
    public double getPos(){
        return motor.getCurrentPosition();
    }

    public double getVelo(){
        return motor.getVelocity();
    }

    public Command OutOfFrameGoal(double x){
        return new RunToPosition(controlSystem, motor.getCurrentPosition() - x * 1.25);
    }

    public void setPwr(double x){
        motor.setPower(x);
    }

    public void resetPwr(){
        motor.setPower(0);
    }

    public double getGoal(double bearing){
        return motor.getCurrentPosition() - bearing * 4.53;
    }

    public Command FollowCam(double bearing){
        return new RunToPosition(controlSystem, motor.getCurrentPosition() - bearing * 4.53);
    }

}