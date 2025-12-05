package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.util.LUT;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Intake;
import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Shooter;

@Config
@TeleOp
public class DynamicVeloTest extends CommandOpMode {

    Servo servo;
    Shooter shooter;
    Intake intake;
    public static double pos;
    public static double number;
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

    //.62 Ta: .28
    //.48 Ta: .955
    //.46 Ta: 1.8

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
        telemetry.addData("velo", shooter.getVelo());
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            double Ta = llresult.getTa();
            telemetry.addData("Ta", Ta);
            if (timer.milliseconds() > 60) {
                shooter.setTo(veloLUT.getClosest(Ta));
                timer.reset();
            }
        }
        telemetry.update();
        CommandScheduler.getInstance().run();
    }
}
