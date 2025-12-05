package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Intake extends SubsystemBase {

    private final Motor motor;

    public Intake(final HardwareMap hw){
        motor = new Motor(hw, "intake");
    }

    public void Spin(double pwr){
        motor.set(-pwr);
    }

    public void PwrOff(){
        motor.set(0);
    }
}
