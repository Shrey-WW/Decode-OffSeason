package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.auto.paths.BluePaths;
import org.firstinspires.ftc.teamcode.auto.paths.Paths;
import org.firstinspires.ftc.teamcode.auto.paths.RedPaths;
import org.firstinspires.ftc.teamcode.commands.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.AutoState;
import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.constants.LaunchState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public abstract class AutoBase extends CommandOpMode {

    private static final double TURRET_LIMIT_CW = 4.5;
    private static final double TURRET_LIMIT_CCW = -3.3;
    private static final double TX_TO_TURRET_GAIN = 2.0;

    private static final double SHOOTER_IDLE_VELOCITY = 800;
    private static final double SHOOTER_END_VELOCITY = 500;

    // Subsystems
    protected Intake intake;
    protected Shooter shooter;
    protected Transfer transfer;
    protected Turret turret;
    protected Limelight3A limelight;
    protected Follower follower;

    // Auto config
    private List<LynxModule> Hubs;
    protected AutoType autoType;
    protected Alliance alliance;
    protected Paths paths;
    protected Pose startingPose;
    protected SequentialCommandGroup AutoSequence;

    protected double GoalX, GoalY = 140;
    private int loopCounter = 0;

    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        AutoState.launchstate = LaunchState.IDLE;

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : Hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if (alliance == Alliance.RED) {
            limelight.pipelineSwitch(1);
            GoalX = 140;
            paths = new RedPaths(autoType, follower);
        } else {
            limelight.pipelineSwitch(0);
            paths = new BluePaths(autoType, follower);
            GoalX = 13;
        }
        paths.buildPaths();
        limelight.start();
    }

    public void run() {
        super.run();
        follower.update();

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));

        for (LynxModule hub : Hubs) {
            hub.clearBulkCache();
        }

        if (loopCounter % 8 == 0) {
            Pose currentPose = follower.getPose();
            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading", currentPose.getHeading());
            telemetry.addData("turret heading", turret.getPosDeg());
            telemetry.addData("balls shot", AutoShootingCMD.numBallsShot);
            telemetry.update();
        }
        loopCounter++;
    }

    protected void updateShooter() {
        if (AutoState.launchstate == LaunchState.IDLE) {
            shooter.setVelocity(SHOOTER_IDLE_VELOCITY);
        } else if (AutoState.launchstate == LaunchState.END) {
            shooter.setVelocity(SHOOTER_END_VELOCITY);
            turret.TurnTo(0);
        }
    }

    /**
     * Autonomous Rotational Correction — tracks the Limelight target by
     * converting tx offset into a turret position adjustment via PID.
     */
    protected void ARC() {
        double turretPos = turret.getPosTicks();

        // Hard safety cutoff regardless of mode
        if (turretPos >= TURRET_LIMIT_CW || turretPos <= TURRET_LIMIT_CCW) {
            turret.pwrOff();
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double txRadians = -Math.toRadians(result.getTx());
            double targetPosition = txRadians * TX_TO_TURRET_GAIN + turretPos;
            if (targetPosition >= TURRET_LIMIT_CW || targetPosition <= TURRET_LIMIT_CCW) {
                turret.pwrOff();
            } else {
                turret.TurnTo(targetPosition);
            }
        }
    }

    protected void OdomTracking() {
        Pose cPos = follower.getPose();
        double dy = GoalY - cPos.getY();
        double dx = GoalX - cPos.getX();
        double CorrectedHeading = Math.atan2(dy, dx);
        double TargetTurretRad = angleWrap(CorrectedHeading - cPos.getHeading());
        double TargetTurretDeg = Math.toDegrees(TargetTurretRad);

        if (!(Math.abs(turret.getPosDeg()) >= 90 && Math.abs(TargetTurretDeg) >= 90))
            turret.TurnTo(TargetTurretDeg);
    }

    private double angleWrap(double radians) {
        if (radians > Math.PI) radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
