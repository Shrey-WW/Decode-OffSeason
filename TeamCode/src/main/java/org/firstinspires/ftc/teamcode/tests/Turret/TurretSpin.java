package org.firstinspires.ftc.teamcode.tests.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Config
public class TurretSpin extends OpMode {

    CRServo servo1, servo2;
    public static double thing1, thing2;
    @Override
    public void init(){
        servo1 = hardwareMap.get(CRServo.class, "turret1");
        servo2 = hardwareMap.get(CRServo.class, "turret2");
    }

    @Override
    public void loop(){
        servo1.setPower(thing1);
        servo2.setPower(thing2);
    }
}
