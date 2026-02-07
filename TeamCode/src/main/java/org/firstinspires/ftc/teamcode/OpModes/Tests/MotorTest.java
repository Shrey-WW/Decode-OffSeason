package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp (group = "tests")
public class MotorTest extends OpMode {

    public static double pwr;
    private DcMotor motor, motor2;
    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "shooter1");
        motor2 = hardwareMap.get(DcMotor.class, "shooter2");
    }

    @Override
    public void loop(){
        motor.setPower(pwr);
        motor2.setPower(pwr);
    }
}
