package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;

public class Turret extends SubsystemBase {

    private CRServoGroup servoGroup;
    private AbsoluteAnalogEncoder encoder;

    SquIDFController squIDFController = new SquIDFController(0,0,0,0);

    public Turret(final HardwareMap hw){
        encoder = new AbsoluteAnalogEncoder(hw, "turretEncoder");
        servoGroup = new CRServoGroup(
                new CRServoEx(hw, "turret1", encoder, CRServoEx.RunMode.RawPower).setCachingTolerance(.1),
                new CRServoEx(hw, "turret2").setCachingTolerance(.1).setRunMode(CRServoEx.RunMode.RawPower));
    }

    public double getPos(){
        return encoder.getCurrentPosition();
    }

    public void set(double speed){
        servoGroup.set(speed);
    }

}
