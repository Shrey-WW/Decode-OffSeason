package org.firstinspires.ftc.teamcode.tests.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.TurretKalmanFilter;

@Config
@TeleOp(group = "tests")
public class TrackTagFused extends CommandOpMode {

    public static double GoalX = 3;
    public static double GoalY = 140;

    public static double Q = 1.0;
    public static double R_odom = 10;
    public static double R_ll = 7;

    private static final double TX_FILTER_ALPHA = 0.4;
    private static final Pose startpose = new Pose(19, 121, Math.toRadians(144));

    private Limelight3A limelight;
    private Turret turret;
    private Follower follower;
    private TurretKalmanFilter kalman;
    private double filteredTx = 0;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startpose);
        follower.startTeleopDrive();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        double initialTarget = Math.toDegrees(angleWrap(
                Math.atan2(GoalY - startpose.getY(), GoalX - startpose.getX()) - startpose.getHeading()
        ));

        kalman = new TurretKalmanFilter(Q, R_odom, R_ll, initialTarget);

        register(turret);
    }

    @Override
    public void run() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        super.run();

        Pose cPos = follower.getPose();
        double odomTarget = Math.toDegrees(angleWrap(
                Math.atan2(GoalY - cPos.getY(), GoalX - cPos.getX()) - cPos.getHeading()
        ));

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult llResult = limelight.getLatestResult();
        Double llTarget = null;
        if (llResult != null && llResult.isValid()) {
            filteredTx = TX_FILTER_ALPHA * llResult.getTx() + (1 - TX_FILTER_ALPHA) * filteredTx;
            llTarget = -filteredTx + turret.getPosDeg();
        }

        kalman.setQ(Q);
        kalman.setRLl(R_ll);
        kalman.setROdom(R_odom);

        double fusedTarget = kalman.estimate(odomTarget, llTarget);
        turret.TurnTo(fusedTarget, Math.toDegrees(follower.getAngularVelocity()));

        telemetry.addData("odom target (deg)", odomTarget);
        telemetry.addData("ll target (deg)", llTarget != null ? llTarget : "no target");
        telemetry.addData("fused target (deg)", fusedTarget);
        telemetry.addData("current pos (deg)", turret.getPosDeg());
        telemetry.update();
    }

    private double angleWrap(double radians) {
        if (radians > Math.PI)  radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}