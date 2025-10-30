package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class TransferServo implements Subsystem {
    public static final TransferServo X = new TransferServo();
    public TransferServo(){}
    ServoEx servo = new ServoEx("transfer");
    private final Command close = new InstantCommand(() -> servo.setPosition(0.85));
    private final Command open = new InstantCommand(() -> servo.setPosition(1));
    
    public Command transfer() {
        if (servo.getPosition() > .9)
            return open;
        return close;
    }
}
