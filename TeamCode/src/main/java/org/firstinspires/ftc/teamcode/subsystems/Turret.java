package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.TurretController;

public class Turret extends SubsystemBase {


    private final CRServoEx servo1, servo2;
    private final MotorEx RevBoreEncoder;
    private static final double TICKS_PER_DEGREE = (double) 69632 / 360;
    private final double pPos = 8.3, iPos = .000006, dPos = 0, pVel = .0015, iVel = 0.000000005, dVel = .00001, kV = .004;

    private final TurretController TurretController = new TurretController(pPos, iPos, dPos, pVel, iVel, dVel, kV);


    public Turret(final HardwareMap hw){
        RevBoreEncoder = new MotorEx(hw , "shooter1");
        servo1 = new CRServoEx(hw, "turret1").setCachingTolerance(.0005);
        servo2 = new CRServoEx(hw, "turret2").setCachingTolerance(.0005);
        servo1.setInverted(true);
        servo2.setInverted(true);
    }

    /**
     @param setPoint Target Position in Degrees
     @param pFF prediction value
    **/
    public void TurnTo(double setPoint, double pFF){
        double output = TurretController.calculate(setPoint, getPosDeg(), getVelocityDegPerSec(), pFF);
        servo1.set(output);
        servo2.set(output);
    }

    /**
     @param setPoint Target Position in Degrees
     **/
    public void TurnTo(double setPoint){
        double output = TurretController.calculate(setPoint, getPosDeg(), getVelocityDegPerSec());
        servo1.set(output);
        servo2.set(output);
    }

    /**
     * @param pwr Power to send Servos
     */
    public void setTo(double pwr){
        servo2.set(pwr);
        servo1.set(pwr);
    }

    /**
     * Sets the servo powers to 0
     */
    public void pwrOff(){
        servo1.set(0);
        servo2.set(0);
    }

    /**
     * @return returns the position of the turret in Ticks
     */
    public double getPosTicks(){
        return RevBoreEncoder.getCurrentPosition();
    }
    /**
     * @return returns the position of the turret in Degrees
     */
    public double getPosDeg(){
        return RevBoreEncoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * @return returns the velocity of the turret in Ticks/s
     */
    public double getVelocityTicksPerSec(){
        return RevBoreEncoder.getVelocity();
    }
    /**
     * @return returns the velocity of the turret in Degrees/s
     */
    public double getVelocityDegPerSec(){
        return RevBoreEncoder.getVelocity() / TICKS_PER_DEGREE;
    }

    public void resetEncoder(){
        RevBoreEncoder.stopAndResetEncoder();
    }

}