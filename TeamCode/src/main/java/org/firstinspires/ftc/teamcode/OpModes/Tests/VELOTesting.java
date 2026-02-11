package org.firstinspires.ftc.teamcode.OpModes.Tests;

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
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;

@Config
@TeleOp (group = "tests")
public class VELOTesting extends CommandOpMode {

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


    @Override
    public void initialize(){
        number = 0;
        veloLUT.add(.18, .67);
        veloLUT.add(.2, .6);
        veloLUT.add(.305, .59);
        veloLUT.add(.45, .5075);
        veloLUT.add(.69, .475);
        veloLUT.add(1.05, .4467);
        veloLUT.add(1.61,.43);
        veloLUT.add(2.81, .424);
        veloLUT.add(5.5, .403);
        veloLUT.createLUT();

        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LutMode = true;
        limelight.start();
    }

    @Override
    public void run() {
        intake.Spin(1);
        transfer.Spin(1);
        if (limelight.getLatestResult().getTa() < .18) {
            shooter.moveServo(0);
        }
        else if (LutMode) {
            shooter.moveServo(.8);
            shooter.setTo(veloLUT.get(limelight.getLatestResult().getTa()));
        }
        else {
            shooter.setTo(number);
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            double Ta = llresult.getTa();
            telemetry.addData("Ta", Ta);
        }

        telemetry.addData("velo", shooter.getVelo());
        telemetry.update();
        super.run();
    }
}
