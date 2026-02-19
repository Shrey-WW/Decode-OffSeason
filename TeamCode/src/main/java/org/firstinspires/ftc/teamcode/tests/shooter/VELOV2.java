package org.firstinspires.ftc.teamcode.tests.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;


@Config
@TeleOp
public class VELOV2 extends CommandOpMode {

    public static double velo;

    private static final double HEIGHT_LIMELIGHT = 16.5;
    private static final double LIMELIGHT_MOUNT_ANGLE = 12.68;
    private static final double HEIGHT_OF_APRILTAG = 29.5;

    private Shooter shooter;
    Transfer transfer;
    Intake intake;
    IMU imu;

    InterpLUT veloLUT = new InterpLUT();
    private Limelight3A limelight;

    private static final double GOAL_X = 13;
    private static final double GOAL_Y = 135;
    private double lastPwr = Double.NaN;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        veloLUT.add(24, 1000);
        veloLUT.add(50, 1100);
        veloLUT.add(67, 1220);
        veloLUT.add(90, 1390);
        veloLUT.add(110, 1480);
        veloLUT.add(150, 1860);
        veloLUT.createLUT();

        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        shooter.moveServo(.8);
        limelight.pipelineSwitch(0);
        intake.Spin(1);
        transfer.Spin(1);
        limelight.start();
    }

    @Override
    public void run() {
        super.run();
        LLResult llresult = limelight.getLatestResult();
        shooter.setVelocity(velo);
        if (llresult != null && llresult.isValid()) {
            telemetry.addData("distance from tag", getDistanceFromTag(llresult));
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Ta", llresult.getTa());
        }
        telemetry.addData("shooter velo", velo);
        telemetry.addData("shooter velo", shooter.getVelo());
        telemetry.update();
    }

    private double getDistanceFromTag(LLResult lr){
        return (HEIGHT_OF_APRILTAG - HEIGHT_LIMELIGHT) / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + lr.getTy()));
    }
}
