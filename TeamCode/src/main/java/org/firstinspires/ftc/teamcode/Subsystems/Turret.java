package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret extends SubsystemBase {


    private CRServoEx servo1, servo2;
    private AbsoluteAnalogEncoder encoder;
    private double kP = 1.15, kI = .005, kD = .027, kF = .012;
    private double truePosition;
    private double lastPos;

    PIDFController PID = new PIDFController(kP,kI, kD, kF);


    public Turret(final HardwareMap hw){
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
        if (setPoint < truePosition);{
            kF = .02;
            PID.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
            kF = .12;
        }
        double output = PID.calculate(truePosition, setPoint);
        servo1.set(output);
        servo2.set(output);
        PID.setF(kF);
    }

    public void setTo(double pwr){
        servo2.set(pwr);
        servo1.set(pwr);
    }

    public void resetPos(){
        truePosition = 0;
    }

    public void pwrOff(){
        servo1.set(0);
        servo2.set(0);
    }

    @Override
    public void periodic(){
        measureTruePos();
    }

    public double getPos(){
        return truePosition;
    }

    public double getTurretHeadingRad(){
        return truePosition * 0.383121055316;
    }

    public double getTurretHeadingDeg(){
        return Math.toDegrees(truePosition * 0.383121055316);
    }

    public void measureTruePos(){
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
