package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.Requirements.AdvancingCommand;

public class Transfer extends SubsystemBase {

    private final ServoEx transfer;
    private final Motor transferMotor;

    public final Command transferCMD;
    public Command close, open;
    public InstantCommand SpinIn, SpinOut, StopTranfser;

    public Transfer(final HardwareMap hw){
        transfer = new ServoEx(hw, "transfer");
        transferMotor = new Motor(hw, "transferMotor");
        close = new InstantCommand(() -> transfer.set(.85));
        open = new InstantCommand(() -> transfer.set(1));
        transferCMD = new AdvancingCommand(open, close);
        SpinIn = new InstantCommand(() -> Spin(1));
        SpinOut = new InstantCommand(()-> Spin(-1));
        StopTranfser = new InstantCommand(this::PwrOff);
    }

    public void setPos(double pos){
        transfer.set(pos);
    }

    public void Spin(double pwr){
        transferMotor.set(-pwr);
    }

    public void PwrOff(){
        transferMotor.set(0);
    }
}
