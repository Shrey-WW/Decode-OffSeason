package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Turret implements Subsystem {
    public static final Turret X = new Turret();
    private Turret() { }

    private final MotorEx motor = new MotorEx("turret");
    // pos: p = .0015 i = 0 d = 0
    // velo:  p =  i =  d =

    public static double p ,i, d;
    private double pwr = 0;

    public Command SpeedUp = new InstantCommand(() -> {
        pwr += .04;
        motor.setPower(pwr);
    }
    );
    public Command SlowDown = new InstantCommand(() -> {
        pwr -= .04;
        motor.setPower(pwr);
    }
    );

    private PIDCoefficients PIDGains = new PIDCoefficients(p,i,d);

    private ControlSystem controlSystem = ControlSystem.builder()
            .posSquid(.00001, 0.000000000005, 10)
            .basicFF(0,0,.12)
            .build();

    @Override
    public void periodic(){
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

    public Command runTo(double goal){
        return new RunToVelocity(controlSystem, goal).requires(this);
    }

    public Command TurnTo(double goal){
        return new RunToPosition(controlSystem, goal).requires(this);
    }

    public Command TurnToGoal(double botHeading){
        double TurretTarget = 62.5 - botHeading;
        double TargetTicks = (2403.125 / 360) * TurretTarget;
        return new RunToPosition(controlSystem, TargetTicks).requires(this);
    }

    public void resetPwr(){
        motor.setPower(0);
    }

    public Command FollowCam(double bearing){
        return new RunToVelocity(controlSystem, bearing * -20);
    }

    public void PIDReset(){
        controlSystem = ControlSystem.builder()
                .posSquid(0,0,0)
                .basicFF(0,0,0)
                .build();
    }

    public void velPID(){
        PIDGains = new PIDCoefficients(p,i,d);
        controlSystem = ControlSystem.builder()
                .velSquID(.000001, 0.000000000005, 10)
                .basicFF(0,0,.11)
                .build();
    }

    public void posPID(){
        controlSystem = ControlSystem.builder()
                .posSquid(.0015)
                .basicFF(0,0,.11)
                .build();
    }

    public double getPos(){
        return motor.getCurrentPosition();
    }

    public PIDCoefficients getPIDGains(){
        return PIDGains;
    }

    public double getVelo(){
        return motor.getVelocity();
    }

}