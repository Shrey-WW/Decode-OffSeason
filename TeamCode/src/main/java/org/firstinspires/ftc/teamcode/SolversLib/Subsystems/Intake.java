package org.firstinspires.ftc.teamcode.SolversLib.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Intake extends SubsystemBase {

    private final Motor motor;
    public InstantCommand SpinIn, SpinOut, StopIntake;

    public Intake(final HardwareMap hw){
        motor = new Motor(hw, "intake");
        SpinIn = new InstantCommand(() -> Spin(1));
        SpinOut = new InstantCommand(() -> Spin(-1));
        StopIntake = new InstantCommand(this::PwrOff);
    }

    public void Spin(double pwr){
        motor.set(pwr);
    }

    public void PwrOff(){
        motor.set(0);
    }

    public double getPower() {
        return motor.get();
    }
}
