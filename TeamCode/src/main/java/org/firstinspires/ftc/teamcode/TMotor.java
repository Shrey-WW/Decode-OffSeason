package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class TMotor implements Subsystem {
    public static final TMotor INSTANCE = new TMotor();
    private TMotor() { }

    private MotorEx motor = new MotorEx("bl");
// p: .01 i: .016
    public static double p,i,d;

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(.008, 0, 0).build();

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

    public Command SpinTo(int goal){
        return new RunToPosition(controlSystem, goal).requires(this);
    }
    public double getPos(){
        return motor.getCurrentPosition();
    }

    public double getGoal(double bearing){
        return motor.getCurrentPosition() - bearing * 4.7;
    }

    public Command FollowCam(double bearing){
        return new RunToPosition(controlSystem, motor.getCurrentPosition() - bearing * 4.7);
    }

}