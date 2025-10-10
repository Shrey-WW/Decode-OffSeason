package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TrainTest extends OpMode {
    DcMotor fL, fR, bL, bR;
    @Override
    public void init(){
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fr");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "br");
    }

    @Override
    public void loop(){
        if (gamepad1.a)
            fL.setPower(1);
        else if (gamepad1.b)
            fR.setPower(1);
        else if (gamepad1.x)
            bL.setPower(1);
        else if (gamepad1.y)
            bR.setPower(1);
        else{
            fR.setPower(0);
            fL.setPower(0);
            bR.setPower(0);
            bL.setPower(0);
        }
    }
}
