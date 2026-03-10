package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorSimple;

public class Intake {

    public enum IntakeStates {
        INTAKING,
        REVERSE,
        REST,
        OFF
    }

    private static IntakeStates currentState = IntakeStates.OFF;

    private CachingDcMotorSimple motor;

    public Intake (@NonNull HardwareMap hw){
        motor = new CachingDcMotorSimple(hw.get(DcMotorSimple.class, "intake"));
        motor.setCachingTolerance(.005);
    }

    public void update() {
        switch (currentState) {
            case INTAKING:
                motor.setPower(1);
                break;

            case REVERSE:
                motor.setPower(-1);
                break;


            case REST:
                motor.setPower(.7);
                break;

            case OFF:
                motor.setPower(0);
                break;
        }
    }

    public void Intaking(){
        currentState = IntakeStates.INTAKING;
    }

    public void Reverse(){
        currentState = IntakeStates.REVERSE;
    }

    public void RestIntake(){
        currentState = IntakeStates.REST;
    }

    public void Off() {
        currentState = IntakeStates.OFF;
    }

}
