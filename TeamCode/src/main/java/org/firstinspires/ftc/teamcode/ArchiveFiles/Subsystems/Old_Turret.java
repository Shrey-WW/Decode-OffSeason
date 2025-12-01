package org.firstinspires.ftc.teamcode.ArchiveFiles.Subsystems;

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
public class Old_Turret implements Subsystem {
    public static final Old_Turret X = new Old_Turret();
    private Old_Turret() { }

    private final MotorEx motor = new MotorEx("turret");
    // pos: kP = .0015 Ki = 0 Kd = 0
    // velo:  kP = .000001  Ki = 0.000000000005  Kd = 10

    public static double kP, Ki, Kd, Ks;
    private double pwr = 0;
    private double cPos = 0;

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

    public PIDCoefficients PIDGains = new PIDCoefficients(kP, Ki, Kd);

    private ControlSystem controlSystem = ControlSystem.builder()
            .posSquid(.00001, 0.000000000005, 1)
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

    public Command TurnRight(){
        cPos += 2403.125/360 * 5;
        return new RunToPosition(controlSystem, cPos).requires(this);
    }

    public Command TurnLeft(){
        cPos -= 2403.125/360 * 5;
        return new RunToPosition(controlSystem, cPos).requires(this);
    }

    public void resetPwr(){
        motor.setPower(0);
    }

    public Command FollowCam(double bearing){
        return new RunToVelocity(controlSystem, bearing * -20);
    }

    public Command resetTicks(){
        return new InstantCommand(() -> motor.setCurrentPosition(0));
    }

    public void PIDReset(){
        controlSystem = ControlSystem.builder()
                .posSquid(0,0,0)
                .basicFF(0,0,0)
                .build();
    }

    public void velPID(){
        PIDGains = new PIDCoefficients(kP, Ki, Kd);
        controlSystem = ControlSystem.builder()
                .velSquID(kP, Ki, Kd)
                .basicFF(0,0, Ks)
                .build();
    }

    public void posPID(){
        PIDGains = new PIDCoefficients(kP, Ki, Kd);
        controlSystem = ControlSystem.builder()
                .posSquid(.0015)
                .basicFF(0,0,0)
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