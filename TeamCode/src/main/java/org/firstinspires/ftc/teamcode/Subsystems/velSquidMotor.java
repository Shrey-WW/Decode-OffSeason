package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class velSquidMotor implements Subsystem {
    public static final velSquidMotor X = new velSquidMotor();
    private velSquidMotor() { }

    private MotorEx motor = new MotorEx("bl");
    public static double p,i,d;

    private PIDCoefficients PIDGains = new PIDCoefficients(.00001,0.000000000005,10);

    private ControlSystem controlSystem = ControlSystem.builder()
            .velSquID(PIDGains)
            .basicFF(0,0,.091)
            .build();

    @Override
    public void periodic(){
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

    public Command runTo(double goal){
        return new RunToVelocity(controlSystem, goal).requires(this);
    }

    public Command SpinTo(double goal){
        return new RunToPosition(controlSystem, goal).requires(this);
    }

    public double getPos(){
        return motor.getCurrentPosition();
    }

    public PIDCoefficients getPIDGains(){
        return PIDGains;
    }

    public void PIDReset(){
        controlSystem = ControlSystem.builder()
                .velSquID(0,0,0)
                .basicFF(0,0,0)
                .build();
    }

    public void velPID(){
        controlSystem = ControlSystem.builder()
                .velSquID(PIDGains)
                .basicFF(0,0,.091)
                .build();
    }

    public void posPID(){
        controlSystem = ControlSystem.builder()
                .posSquid(.00573, .000000000001, 0.00000033)
                .build();
    }

    public double getGoal(double bearing){
        return motor.getCurrentPosition() - bearing * 4.53;
    }

    public void resetPwr(){
        motor.setPower(0);
    }

    public double getVelo(){
        return motor.getVelocity();
    }

    public Command FollowCam(double bearing){
        return new RunToVelocity(controlSystem, bearing * -20);
    }

}