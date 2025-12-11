package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//WORKS AS OF NOV 10 PRETTY GOOD WOULD RAT LIKE 6/7
@TeleOp (group = "tests")
public class limelight extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private final double scale = 72.6571;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            Pose3D botPose = llresult.getBotpose_MT2();
            telemetry.addData("distance", getDistanceFromTag(llresult.getTa()));
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Ta", llresult.getTa());
            telemetry.addData("BotPose", botPose.toString());
        }
        telemetry.update();
    }

    public double getDistanceFromTag(double ta){
        double distance = scale * Math.pow(ta, -0.506577);
        return distance;
    }
}
