package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

public class ServoTurret extends SubsystemBase {
    private CRServoEx Servo1, Servo2;

    private CRServoEx servoGroup;

    public ServoTurret(final HardwareMap hw){
    }
}
