package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Config
@TeleOp
public class VELOV2 extends CommandOpMode {

    public static double pwr;

    private Shooter shooter;
    private Follower follower;
    Transfer transfer;
    Intake intake;
    IMU imu;

    private Limelight3A limelight;

    private static final double GOAL_X = 13;
    private static final double GOAL_Y = 135;
    private double lastPwr = Double.NaN;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        follower.update();
        super.run();

        shooter.setTo(pwr);

        Pose currentPose = follower.getPose();

        telemetry.addData("distance to goal", getDistanceFromTag(limelight.getLatestResult()));
        telemetry.addData("shooter pwr", pwr);
        telemetry.addData("shooter velo", shooter.getVelo());
        telemetry.update();
    }


    public double getDistanceFromTag(LLResult llresult) {
        List<LLResultTypes.FiducialResult> r = llresult.getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i : r) {
            if (i != null && i.getFiducialId() == 20) {
                target = i;
                break;
            }
        }

        if (target != null) {
            double x = (target.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch) + 8; // right/left from tag
            double z = (target.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 8; // forward/back from tag

            Vector e = new Vector();
            e.setOrthogonalComponents(x, z);
            return e.getMagnitude();
        }
        return 0;
    }
}
