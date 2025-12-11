package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp (group = "tests")
public class AutoAlignTest extends OpMode {

    Limelight3A limelight;
    IMU imu;
    Turret turret;
    private static final int TICKS_PER_REV = 2403;
    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        turret = new Turret(hardwareMap);
        turret.setPositionControl();
    }

    @Override
    public void start(){
        limelight.start();
    }


    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid() & gamepad1.left_trigger > .1) {
            autoAlign(llresult.getTx());
        }
    }


    private void autoAlign(double Tx){
        int cPos = turret.getPos();
        turret.goToPos((int) (cPos + TICKS_PER_REV * Tx/360));
    }
}
