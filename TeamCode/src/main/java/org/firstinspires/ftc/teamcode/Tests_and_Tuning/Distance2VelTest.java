package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@TeleOp
public class Distance2VelTest extends NextFTCOpMode {
    private Limelight3A limelight;
    private IMU imu;
    private static final double scale = 72.6571;
    public static double targetVel;
    public static double hoodPos;
    @Override
    public void onInit(){
        addComponents(
                new SubsystemComponent(Shooter.X)
        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void onStartButtonPressed(){
        limelight.start();
    }

    @Override
    public void onUpdate(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();

        Shooter.X.setVelocity(targetVel).schedule();
        Shooter.X.setHood(hoodPos).schedule();
        if (llresult != null && llresult.isValid()) {
            telemetry.addData("distance", getDistanceFromTag(llresult.getTa()));
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ta", llresult.getTa());
        }
        telemetry.update();
    }


    public double getDistanceFromTag(double ta){
        double distance = scale * Math.pow(ta, -0.506577);
        return distance;
    }
}
