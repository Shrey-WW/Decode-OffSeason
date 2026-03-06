package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.auto.paths.BluePaths;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.paths.RedPaths;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.AutoState;
import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.TurretKalmanFilter;

import java.util.List;

public abstract class AutoBase extends CommandOpMode {

    private static final double TX_FILTER_ALPHA = 0.4;
    protected double filteredTx;

    public static double Q = 1;
    public static double R_odom = 10;
    public static double R_ll = 7;
    protected static double SHOOTER_IDLE_VELOCITY = 1000;
    protected static double SPIN_UP_VELOCITY = 1100;
    protected static final double SHOOTER_END_VELOCITY = 500;


    // Subsystems
    protected Intake intake;
    protected Shooter shooter;
    protected Transfer transfer;
    protected Turret turret;
    protected Limelight3A limelight;
    protected VoltageSensor voltageSensor;
    protected Follower follower;

    // Auto config
    private TurretKalmanFilter kalman;
    private List<LynxModule> Hubs;
    protected AutoType autoType;
    protected Alliance alliance;
    protected Paths paths;
    protected Pose startingPose;
    protected SequentialCommandGroup AutoSequence;
    private final ElapsedTime looptimes = new ElapsedTime();

    protected double GoalX, GoalY = 144;

    public void initialize() {
        looptimes.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : Hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if (alliance == Alliance.RED) {
            limelight.pipelineSwitch(1);
            GoalX = 144;
            paths = new RedPaths(autoType, follower);
        } else {
            limelight.pipelineSwitch(0);
            paths = new BluePaths(autoType, follower);
            GoalX = 0;
        }
        paths.buildPaths();
        limelight.start();

        double initialTarget = Math.toDegrees(angleWrap(
                Math.atan2(GoalY - startingPose.getY(), GoalX - startingPose.getX()) - startingPose.getHeading()
        ));

        kalman = new TurretKalmanFilter(Q, R_odom, R_ll, initialTarget);
        turret.resetEncoder();
    }

    public void run() {
        super.run();
        follower.update();

        for (LynxModule hub : Hubs) {
            hub.clearBulkCache();
        }

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));

        telemetry.addData("loop time ", looptimes.milliseconds());
        telemetry.addData("numBallsShot ", AutoShootingCMD.numBallsShot);
        looptimes.reset();
        telemetry.update();
    }

    protected void updateShooter() {
        switch (AutoState.launchstate) {
            case IDLE:
                shooter.setVelocity(SHOOTER_IDLE_VELOCITY);
                break;

            case END:
                shooter.setVelocity(SHOOTER_END_VELOCITY);
                break;

            case SPIN_UP:
                shooter.setVelocity(SPIN_UP_VELOCITY);
                break;
        }
    }

    /**
     * April-Tag Rotational Correction
     */
    protected void ARC() {
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

        double fusedTarget = kalman.estimate(odomTarget, llTarget);
        turret.TurnTo(fusedTarget);
    }

    private double angleWrap(double radians) {
        if (radians > Math.PI) radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
