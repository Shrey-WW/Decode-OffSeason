package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Shooter implements Subsystem {
    private Shooter () {}
    private MotorEx motor = new MotorEx("br");
    private ServoEx servo = new ServoEx("arm1");

    public void speed(double h){
        motor.setPower((318.31*Math.sqrt(19.6*h))/2000);
    }
    public void angle(double h)
    {
        servo.setPosition(Math.asin(1/h));
    }

}
