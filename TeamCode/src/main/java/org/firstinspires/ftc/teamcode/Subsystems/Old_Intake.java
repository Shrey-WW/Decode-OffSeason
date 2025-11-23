package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;

import dev.nextftc.hardware.impl.MotorEx;

import dev.nextftc.core.subsystems.Subsystem;

public class Old_Intake implements Subsystem {
    public static final Old_Intake X = new Old_Intake();
    private final MotorEx motor = new MotorEx("intake");

    private Old_Intake(){}

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