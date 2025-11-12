package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp (group = "tests")
public class motortest extends OpMode {
    public static double pwr;
    private DcMotor motor;
    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "turret");
    }

    @Override
    public void loop(){
        motor.setPower(pwr);
        telemetry.update();
    }

}
