package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

public class Turret extends SubsystemBase {


    private CRServoEx servo1, servo2;
    private AbsoluteAnalogEncoder encoder;
    private double truePosition;
    private double lastPos;

    PIDFController PID = new PIDFController(0.8,0,0,0.001);

    public Turret(final HardwareMap hw){
        encoder = new AbsoluteAnalogEncoder(hw, "turretEncoder");
        servo1 = new CRServoEx(hw, "turret1").setCachingTolerance(.0005);
        servo2 = new CRServoEx(hw, "turret2").setCachingTolerance(.0005);
    }

    /**
     @param setPoint Target Position
     pFF prediction value
    **/
    public void PIDto(double setPoint){
        double output = PID.calculate(truePosition, setPoint);
        servo1.set(output);
        servo2.set(output);
    }

    public void setTo(double pwr){
        servo2.set(pwr);
        servo1.set(pwr);
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
