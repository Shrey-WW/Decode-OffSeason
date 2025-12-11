package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.LUT;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@Config
@TeleOp (group = "tests")
public class DynamicVeloTest extends CommandOpMode {

    Shooter shooter;
    Intake intake;
    public static double number;
    public static double pos;
    private Limelight3A limelight;
    ElapsedTime timer = new ElapsedTime();
    private IMU imu;
    LUT<Double, Double> veloLUT = new LUT<Double, Double>()
    {{
        add(.28, .62);
        add(.5, .3);
        add(1.0,.48);
        add(2.5, .46);
    }};

    //.53 Ta: .632
    //.45 Ta: 1.7976140603423119
    //.42 Ta: 2.262764237821102

    @Override
    public void initialize(){
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.start();
    }

    @Override
    public void run() {
        intake.Spin(1);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            double Ta = llresult.getTa();
            telemetry.addData("Ta", Ta);
        }
        shooter.moveServo(pos);
        shooter.setTo(number);

        telemetry.addData("velo", shooter.getVelo());
        telemetry.update();
        super.run();
    }
}
