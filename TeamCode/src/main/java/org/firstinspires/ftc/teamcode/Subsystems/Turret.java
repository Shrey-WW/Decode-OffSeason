package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

public class Turret {

    public enum TurretStates {
        TRACKING,
        POSITION_CONTROL,
        OFF
    }

    private static TurretStates currentState = TurretStates.OFF;
    private CachingCRServo turret1, turret2;

    public Turret(HardwareMap hw){
        turret1 = new CachingCRServo(hw.get(CRServo.class, "turret1"));
        turret2 = new CachingCRServo(hw.get(CRServo.class, "turret2"));

        turret1.setCachingTolerance(.005);
        turret1.setDirection(DcMotorSimple.Direction.REVERSE);
        turret2.setCachingTolerance(.005);
        turret2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void update(){
        switch(currentState){

            case TRACKING:
                break;

            case POSITION_CONTROL:
                break;

            case OFF:
                turret1.setPower(0);
                turret1.setPower(0);
                break;
        }
    }
}
