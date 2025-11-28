package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Old_Shooter;

@Config
@TeleOp (group = "tests")
public class motortest extends OpMode {
    public static double pwr;
    private DcMotor motor;
    private DcMotor motor2;
    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "shooter1");
        motor2 = hardwareMap.get(DcMotor.class, "shooter2");
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop(){
        motor.setPower(pwr);
        motor2.setPower(pwr);
        telemetry.addData("velo", Old_Shooter.X.getVelo());
        telemetry.update();
    }

}
