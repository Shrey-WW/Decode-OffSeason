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
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.LUT;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@Config
@TeleOp (group = "tests")
public class DynamicVeloTest extends CommandOpMode {

    Shooter shooter;
    Intake intake;
    public static double number;
    public static double pos;
    public static Boolean LutMode;

    Transfer transfer;
    private Limelight3A limelight;
    ElapsedTime timer = new ElapsedTime();
    private IMU imu;
    InterpLUT veloLUT = new InterpLUT();

    //.53 Ta: .632
    //.45 Ta: 1.7976140603423119
    //.42 Ta: 2.262764237821102

    @Override
    public void initialize(){
        veloLUT.add(.28, .62);
        veloLUT.add(.623, .53);
        veloLUT.add(1.0,.48);
        veloLUT.add(2.26, .42);
        veloLUT.createLUT();

        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        limelight.start();
    }

    @Override
    public void run() {
        intake.Spin(1);
        transfer.Spin(1);
        if (LutMode){
            shooter.setTo(0);
        }
        else {
            shooter.setTo(number);
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        if (true)
//        limelight.updateRobotOrientation(orientation.getYaw());
//        LLResult llresult = limelight.getLatestResult();

//        if (llresult != null && llresult.isValid()) {
//            double Ta = llresult.getTa();
//            telemetry.addData("Ta", Ta);
//        }

        telemetry.addData("velo", shooter.getVelo());
        telemetry.update();
        super.run();
    }
}
