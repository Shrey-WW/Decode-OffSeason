package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp (group = "tests")
public class HoodServoTest extends OpMode {

    Servo servo;
    public static double pos;

    @Override
    public  void init() {
        servo = hardwareMap.get(Servo.class, "hood");
    }

    @Override
    public void loop() {
        servo.setPosition(pos);
    }
    
}
