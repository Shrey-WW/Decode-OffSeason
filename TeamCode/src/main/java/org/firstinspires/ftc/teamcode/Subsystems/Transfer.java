package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.CustomCMDs.AdvancingCommand;

public class Transfer extends SubsystemBase {
    private final ServoEx transfer;
    public final Command transferCMD;
    public Command close, open;
    public Transfer(final HardwareMap hw){
        transfer = new ServoEx(hw, "transfer");
        close = new InstantCommand(() -> transfer.set(.91));
        open = new InstantCommand(() -> transfer.set(1));
        transferCMD = new AdvancingCommand(open, close);
    }
}
