package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.CCmds.SwitchCMD;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Old_Transfer implements Subsystem {
    public static final Old_Transfer X = new Old_Transfer();
    public Old_Transfer(){}
    ServoEx servo = new ServoEx("transfer");

    public final Command close = new InstantCommand(() -> servo.setPosition(0.91));
    public final Command open = new InstantCommand(() -> servo.setPosition(1));
    public final SwitchCMD transfer = new SwitchCMD(open, close);
}
