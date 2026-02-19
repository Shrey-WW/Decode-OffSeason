package org.firstinspires.ftc.teamcode.tests.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class VelMotor extends OpMode {

    DcMotor motor;
    DcMotor motor2;
    @Override
    public void init(){
        motor = hardwareMap.get(DcMotorEx.class, "shooter2");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooter1");
    }

    @Override
    public void loop(){
        motor.setPower(.3);
        motor2.setPower(.3);
    }
}
