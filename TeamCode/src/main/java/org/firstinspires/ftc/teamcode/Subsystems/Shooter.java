package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Shooter extends SubsystemGroup {
    public static final Shooter X = new Shooter();
    private Shooter() {
        super(
                ShootMotor.X,
                AngleServo.X
        );
    }

    private MotorEx motor = new MotorEx("br");
    private ServoEx servo = new ServoEx("arm1");

    public void speed(double h){
        motor.setPower((318.31*Math.sqrt(19.6*h))/6000);
    }
    public void angle(double h)
    {
        servo.setPosition(Math.asin(1/h));
    }

}

class ShootMotor implements Subsystem {
    public static final ShootMotor X = new ShootMotor();
    private ShootMotor() {}

    private MotorEx motor = new MotorEx("br");
    private ControlSystem controlSystem = ControlSystem.builder()
            .velSquID(.00573, .000000000001, 0.00000033)
            .build();
    @Override
    public void periodic(){
        motor.setPower(controlSystem.calculate(motor.getState()));
    }
}

class AngleServo implements Subsystem {
    public static final AngleServo X = new AngleServo();
    private AngleServo(){}
    private ServoEx servo = new ServoEx("arm1");

    public Command tiltTo(double pos){
        return new SetPosition(servo, pos).requires(this);
    }
}