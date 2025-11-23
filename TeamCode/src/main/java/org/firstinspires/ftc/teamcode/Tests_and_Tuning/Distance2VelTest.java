package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Old_Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@TeleOp (group = "tests")
public class Distance2VelTest extends NextFTCOpMode {
    private Limelight3A limelight;
    private IMU imu;
    private static final double scale = 72.6571;
    public static double targetVel;
    public static double hoodPos;
    Robot bot;
    @Override
    public void onInit(){
        addComponents(
                new SubsystemComponent(Shooter.X, Old_Intake.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = new Robot(this);

    }

    @Override
    public void onStartButtonPressed(){
        bot.drive.schedule();
        limelight.start();
        Old_Intake.X.SpinIn(1).schedule();
    }

    @Override
    public void onUpdate(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        double Ta = llresult.getTa();
        Shooter.X.setHood(hoodPos).schedule();
        if (llresult != null && llresult.isValid()) {
            telemetry.addData("distance", getDistanceFromTag(llresult.getTa()));
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ta", Ta);
            telemetry.addData("vel", Shooter.X.getVelo());
            telemetry.addData("target velo", calcVelocity(Ta));
        }
        telemetry.update();
        Shooter.X.setVelocity(targetVel).schedule();
    }


    public double getDistanceFromTag(double ta){
        double distance = scale * Math.pow(ta, -0.506577);
        return distance;
    }

    public double calcVelocity(double Ta){
        return 1470 * Math.pow(Ta, -.141712);
    }
}
