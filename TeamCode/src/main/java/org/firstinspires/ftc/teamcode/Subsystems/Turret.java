package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Turret extends SubsystemBase {

    private CRServoGroup servoGroup;
    private AbsoluteAnalogEncoder encoder;

    SquIDFController squIDFController = new SquIDFController(0,0,0,0);

    public Turret(final HardwareMap hw){
        servoGroup = new CRServoGroup(
                new CRServoEx(hw, "turret1").setCachingTolerance(.1).setRunMode(CRServoEx.RunMode.RawPower),
                new CRServoEx(hw, "turret2").setCachingTolerance(.1).setRunMode(CRServoEx.RunMode.RawPower));
        encoder = new AbsoluteAnalogEncoder(hw, "turretEncoder");
    }

    public double getPos(){
        return encoder.getCurrentPosition();
    }

    public void set(double speed){
        servoGroup.set(speed);
    }

    





}
