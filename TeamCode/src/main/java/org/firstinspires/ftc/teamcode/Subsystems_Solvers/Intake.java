package org.firstinspires.ftc.teamcode.Subsystems_Solvers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake extends SubsystemBase {

    private final Motor motor;

    public Intake(final HardwareMap hw){
        motor = new Motor(hw, "intake");
    }

    public void SpinIn(double pwr){
        motor.set(-pwr);
    }

    public void SpinOut(double pwr){
        motor.set(pwr);
    }

    public void PwrOff(){
        motor.set(0);
    }
}
