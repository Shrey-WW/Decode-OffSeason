package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.CCmds.SwitchCMD;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
@Config
public class Shooter implements Subsystem {
    private Shooter(){}
    //motor 2 HAS ENCODER
    //HOOD SERVO CAN GO BETWEEN .7 and .4
    public static final Shooter X = new Shooter();

    public static double p,i,d;
    MotorEx motor = new MotorEx("shooter1");
    MotorEx motor2 = new MotorEx("shooter2");
    ServoEx Hood = new ServoEx("hood");
    private final MotorGroup ShootingMotors = new MotorGroup(motor2, motor);
    public Command FullPowerShot = new InstantCommand(() -> setPwr(1));
    public Command ShortPowerShot = new InstantCommand(() -> setPwr(.84));
    public Command IncPower = new InstantCommand(() -> setPwr(getPwr() + .02));
    public Command DecPower = new InstantCommand(() -> setPwr(getPwr() - .02));
    public Command HighHood = new InstantCommand(() -> Hood.setPosition(.4));
    public Command LowHood = new InstantCommand(() -> Hood.setPosition(.6));
    public SwitchCMD SwitchHood = new SwitchCMD(HighHood, LowHood);


    private PIDCoefficients PIDGains = new PIDCoefficients(p, i, d);
    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(PIDGains)
            .basicFF(0.0075,.00006,.09)
            .build();

    @Override
    public void periodic(){
        ShootingMotors.setPower(controlSystem.calculate(motor.getState()));
    }

    public Command setVelocity(double goal){
        return new RunToVelocity(controlSystem, goal).requires(this);
    }

    public void setPwr(double pwr){
        ShootingMotors.setPower(pwr);
    }

    public void updatePID(){
        PIDGains = new PIDCoefficients(p, i, d);
        controlSystem = ControlSystem.builder()
            .velPid(PIDGains)
            .basicFF(0.0075,0.0006,.09)
            .build();
    }

    public PIDCoefficients getPIDCoefficients(){
        return PIDGains;
    }

    public double getVelo(){
        return ShootingMotors.getVelocity();
    }

    public double getPos(){
        return ShootingMotors.getCurrentPosition();
    }

    public double getPwr(){
        return ShootingMotors.getPower();
    }

}
