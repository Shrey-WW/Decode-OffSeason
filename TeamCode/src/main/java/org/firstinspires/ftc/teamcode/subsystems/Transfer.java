package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;


public class Transfer extends SubsystemBase {

    private final Motor TMotor;
    public InstantCommand SpinIn, SpinOut, StopTransfer;

    public Transfer(final HardwareMap hw){
        TMotor = new Motor(hw, "TMotor");

        SpinIn = new InstantCommand(() -> Spin(.8));
        SpinOut = new InstantCommand(()-> Spin(-.8));
        StopTransfer = new InstantCommand(this::PwrOff);
    }

    public void Spin(double pwr){
        TMotor.set(-pwr);
    }

    public void PwrOff(){
        TMotor.set(0);
    }
}