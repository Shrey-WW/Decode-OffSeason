package org.firstinspires.ftc.teamcode.Tests_and_Tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp (group = "tests")
public class TrackTagLL extends NextFTCOpMode {
    private Limelight3A limelight;
    private IMU imu;
    ElapsedTime timer = new ElapsedTime();
    private double cPos;
    private double Bearing;
    private Command setVelPID;
    private final double TICKS_PER_DEGREES = (384.5 * 100 / 16) / 360;
    private LLResult llresult;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(Turret.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void onWaitForStart() {
        Turret.X.resetPwr();
        Turret.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed() {
        Turret.X.posPID();
        limelight.start();
    }

    @Override
    public void onUpdate() {
        long start = System.nanoTime();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            double tx = llresult.getTx();
            TrackTag(tx);
        }
        if (timer.milliseconds() > 100) {
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Current motor pos", Turret.X.getPos());
            telemetry.addData("Current motor vel", Turret.X.getVelo());
            telemetry.update();
            timer.reset();
        }
    }

    public void TrackTag(double Tx) {
        double ticks2turn = Tx * TICKS_PER_DEGREES;
        Turret.X.TurnTo(Turret.X.getPos()-ticks2turn);
    }
}
