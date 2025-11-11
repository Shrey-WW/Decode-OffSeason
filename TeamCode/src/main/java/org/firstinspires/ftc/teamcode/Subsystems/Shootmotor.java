package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Shootmotor implements Subsystem {
    private Shootmotor(){}
    public static final Shootmotor X = new Shootmotor();
    MotorEx smotor = new MotorEx("shooter1");
    MotorEx smotor2 = new MotorEx("shooter2");
    public double getVelo(){
        return smotor2.getVelocity();
    }
    public double getVelo2(){
        return smotor.getVelocity();
    }
    public void setPwr(double pwr){
        smotor.setPower(pwr);
        smotor2.setPower(pwr);
    }
}
