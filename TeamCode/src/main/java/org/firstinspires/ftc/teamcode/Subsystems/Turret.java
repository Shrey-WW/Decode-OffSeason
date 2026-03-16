package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.CascadeController;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;

public class Turret {

    public enum TurretStates {
        TRACKING,
        POSITION_CONTROL,
        OFF
    }

    // Turret variables
    private static TurretStates currentState = TurretStates.OFF;
    private static double targetPosition;

    // Hardware
    private CachingCRServo turret1, turret2;
    private DcMotorEx encoder;

    // Turret Constants
    private static final double TICKS_PER_DEGREE = (double) 69632 / 360;
    private static final double
            pPos = 8.3, iPos = .000006, dPos = 0,
            pVel = .0015, iVel = 0.000000005, dVel = .00001,
            kV = .004;

    private final CascadeController TurretController = new CascadeController(pPos, iPos, dPos, pVel, iVel, dVel, kV, 150);

    public Turret(HardwareMap hw){
        turret1 = new CachingCRServo(hw.get(CRServo.class, "turret1"));
        turret2 = new CachingCRServo(hw.get(CRServo.class, "turret2"));
        encoder = hw.get(DcMotorEx.class, "shooter1");

        turret1.setCachingTolerance(.005);
        turret1.setDirection(DcMotorSimple.Direction.REVERSE);
        turret2.setCachingTolerance(.005);
        turret2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(){
        switch(currentState){

            case TRACKING:
                double ARC_TargetPwr = TurretController.calculate(targetPosition, getDegreePosition(), getDegreeVelocity());
                turret1.setPower(ARC_TargetPwr);
                turret2.setPower(ARC_TargetPwr);
                break;

            case POSITION_CONTROL:
                double targetPwr = TurretController.calculate(targetPosition, getDegreePosition(), getDegreeVelocity());
                turret1.setPower(targetPwr);
                turret2.setPower(targetPwr);
                if (Math.abs(targetPosition - encoder.getCurrentPosition()) < 1.2) { currentState = TurretStates.OFF; }
                break;

            case OFF:
                turret1.setPower(0);
                turret1.setPower(0);
                break;
        }
    }

    public void stop() {
        currentState = TurretStates.OFF;
    }

    public void track() {
        currentState = TurretStates.TRACKING;
    }

    public void runToPosition(double pos){
        currentState = TurretStates.POSITION_CONTROL;
        targetPosition = pos;
    }

    public void setTargetPosition(double pos) {
        targetPosition = pos;
    }

    public double getDegreePosition(){
        return encoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public double getDegreeVelocity() {
        return encoder.getVelocity() / TICKS_PER_DEGREE;
    }
}
