package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake X = new Intake();
    private final MotorEx motor = new MotorEx("intake");
    private Intake(){}
    public Command SpinOut(double pwr){
        return new InstantCommand(() -> motor.setPower(pwr));
    }
    public Command SpinIn(double pwr){
        return new InstantCommand(() -> motor.setPower(-pwr));
    }
    public void PwrOff(){
        motor.setPower(0);
    }
}
