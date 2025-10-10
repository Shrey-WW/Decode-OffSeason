package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class flywheelTest extends OpMode {

    private DcMotor motor;
    String lastinput = "none";
    double pwr = 0;

    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "fl");
    }

    @Override
    public void loop(){
        if (lastinput.equals("none") && gamepad1.dpad_up){
            pwr += .05;
            lastinput = "up";
        }
        if (lastinput.equals("none") && gamepad1.dpad_down){
            pwr -= .05;
            lastinput = "down";
        }
        if (!gamepad1.dpad_down && !gamepad1.dpad_up){
            lastinput = "none";
        }
        motor.setPower(pwr);
        telemetry.addData("pwr", pwr);
        telemetry.update();
    }
}
