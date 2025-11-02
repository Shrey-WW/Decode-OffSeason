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
                HoodServo.X
        );
    }

    public void speed(double h){
        ShootMotor.X.motor.setPower((Math.sqrt(19.6 * h)/Math.PI * .192) / 6000);
    }
}

class ShootMotor implements Subsystem {
    public static final ShootMotor X = new ShootMotor();
    private ShootMotor() {}

    public MotorEx motor = new MotorEx("br");
    public MotorEx motor2 = new MotorEx("fr");
    private ControlSystem controlSystem = ControlSystem.builder()
            .velSquID(.00573, .000000000001, 0.00000033)
            .build();

    @Override
    public void periodic(){
        motor2.setPower(controlSystem.calculate(motor.getState()));
        motor.setPower(controlSystem.calculate(motor.getState()));
    }
}

class HoodServo implements Subsystem {
    public static final HoodServo X = new HoodServo();
    private HoodServo(){}
    private ServoEx servo = new ServoEx("arm1");

    public Command tiltTo(double pos){
        return new SetPosition(servo, pos).requires(this);
    }
}