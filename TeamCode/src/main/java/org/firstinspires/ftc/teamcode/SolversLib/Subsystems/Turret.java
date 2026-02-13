package org.firstinspires.ftc.teamcode.SolversLib.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;

public class Turret extends SubsystemBase {


    private final CRServoEx servo1, servo2;
    private final MotorEx RevBoreEncoder;
    private final AbsoluteAnalogEncoder encoder;
    private final double kP = 1, kI = 0, kD = 0, kF = .12;
    private double truePosition;
    private double lastPos;

    PIDFController PID = new PIDFController(kP,kI, kD, kF);


    public Turret(final HardwareMap hw){
        RevBoreEncoder = new MotorEx(hw , "shooter1");
        encoder = new AbsoluteAnalogEncoder(hw, "turretEncoder");
        servo1 = new CRServoEx(hw, "turret1").setCachingTolerance(.0005);
        servo2 = new CRServoEx(hw, "turret2").setCachingTolerance(.0005);
    }

    /**
     @param setPoint Target Position
     @param pFF prediction value
    **/
    public void PIDto(double setPoint, double pFF){
        double output = PID.calculate(truePosition, setPoint);
        servo1.set(output + pFF);
        servo2.set(output + pFF);
    }

    /**
     @param setPoint Target Position
     **/
    public void PIDto(double setPoint){
        PID.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
        double output = PID.calculate(truePosition, setPoint);
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

    public void resetPos(){
        truePosition = 0;
    }

    /**
     * Sets the servo powers to 0
     */
    public void pwrOff(){
        servo1.set(0);
        servo2.set(0);
    }

    @Override
    public void periodic(){
        measureTruePos();
    }

    /**
     * @return returns the true position of the turret
     */
    public double getPos(){
        return truePosition;
    }

    /**
     * @return returns the Turret Heading in Radians
     */
    public double getTurretHeadingRad(){
        return truePosition * 0.383121055316;
    }

    /**
     * @return returns the Turret Heading in Degrees
     */
    public double getTurretHeadingDeg(){
        return Math.toDegrees(truePosition * 0.383121055316);
    }

    private void measureTruePos(){
        double currentPos = MathUtils.normalizeRadians(encoder.getCurrentPosition(), false);

        if (currentPos < 0){
            if (lastPos < 0){
                truePosition += currentPos - lastPos;
            }
            else if (lastPos > 3){
                double delta1 = -(-Math.PI - currentPos);
                double delta2 = Math.PI - lastPos;
                truePosition += delta1 + delta2;
            }

        }
        else if (currentPos >= 0) {
            if (lastPos > 0){
                truePosition += currentPos - lastPos;
            }
            else if(lastPos < -3){
                double delta1 = -(-Math.PI - lastPos);
                double delta2 = Math.PI - currentPos;
                truePosition += delta1 + delta2;
            }
        }
        lastPos = currentPos;
    }

}
