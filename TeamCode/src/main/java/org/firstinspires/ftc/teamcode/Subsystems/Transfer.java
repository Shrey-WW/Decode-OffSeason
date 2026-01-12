package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.Requirements.AdvancingCommand;

public class Transfer extends SubsystemBase {

    private final ServoEx TServo;
    private final Motor TMotor;

    public final Command transferCMD;
    public Command close, open;
    public InstantCommand SpinIn, SpinOut, StopTransfer;

    public Transfer(final HardwareMap hw){
        TServo = new ServoEx(hw, "TServo");
        TMotor = new Motor(hw, "TMotor");
        close = new InstantCommand(() -> TServo.set(.85));
        open = new InstantCommand(() -> TServo.set(1));
        transferCMD = new AdvancingCommand(open, close);

        SpinIn = new InstantCommand(() -> Spin(1));
        SpinOut = new InstantCommand(()-> Spin(-1));
        StopTransfer = new InstantCommand(this::PwrOff);
    }

    public void setPos(double pos){
        TServo.set(pos);
    }

    public void Spin(double pwr){
        TMotor.set(pwr);
    }

    public void PwrOff(){
        TMotor.set(0);
    }
}
