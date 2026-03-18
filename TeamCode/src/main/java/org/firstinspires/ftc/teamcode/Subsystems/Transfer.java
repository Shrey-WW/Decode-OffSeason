package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorSimple;

public class Transfer {

    public enum TransferStates {
        TRANSFERING,
        REVERSE,
        OFF
    }

    private static TransferStates currentState = TransferStates.OFF;

    private CachingDcMotorSimple motor;

    public Transfer(@NonNull HardwareMap hw) {
        motor = new CachingDcMotorSimple(hw.get(DcMotorSimple.class, "transfer"));
        motor.setCachingTolerance(.005);
    }

    public void update() {
        switch (currentState) {
            case TRANSFERING:
                motor.setPower(1);
                break;

            case REVERSE:
                motor.setPower(-0.6);
                break;

            case OFF:
                motor.setPower(0);
                break;
        }
    }

    public void Transfer() {
        currentState = TransferStates.TRANSFERING;
    }

    public void Reverse() {
        currentState = TransferStates.REVERSE;
    }

    public void Off() {
        currentState = TransferStates.OFF;
    }
}
