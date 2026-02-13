package org.firstinspires.ftc.teamcode.Globals.Paradigms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Globals.Paths.BluePaths;
import org.firstinspires.ftc.teamcode.Globals.Paths.RedPaths;
import org.firstinspires.ftc.teamcode.Globals.States.Alliance;
import org.firstinspires.ftc.teamcode.Globals.States.AutoStates;
import org.firstinspires.ftc.teamcode.Globals.States.AutoType;
import org.firstinspires.ftc.teamcode.Globals.States.LaunchState;
import org.firstinspires.ftc.teamcode.SolversLib.CMDs.AutoShootingCMD;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class AutoBase extends CommandOpMode {

    private static final double TURRET_LIMIT_CW = 4.5;
    private static final double TURRET_LIMIT_CCW = -3.3;
    private static final double TX_TO_TURRET_GAIN = 2.0;

    private static final double SHOOTER_IDLE_POWER = 0.35;
    private static final double SHOOTER_END_POWER = 0.2;

    // Subsystems
    protected Intake intake;
    protected Shooter shooter;
    protected Transfer transfer;
    protected Turret turret;
    protected Limelight3A limelight;
    protected Follower follower;

    // Auto config (set by subclass before calling initialize)
    protected AutoType autoType;
    protected Alliance alliance;
    protected Paths paths;
    protected Pose startingPose;
    protected SequentialCommandGroup AutoSequence;

    protected double GoalX, GoalY = 135;
    private int loopCounter = 0;

    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        AutoStates.launchstate = LaunchState.IDLE;

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        if (alliance == Alliance.RED) {
            limelight.pipelineSwitch(1);
            GoalX = 132.5;
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

        updateShooter();

        if (loopCounter % 8 == 0) {
            Pose currentPose = follower.getPose();
            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading", currentPose.getHeading());
            telemetry.addData("turret heading", turret.getTurretHeadingDeg());
            telemetry.addData("balls shot", AutoShootingCMD.numBallsShot);
            telemetry.update();
        }
        loopCounter++;
    }

    private void updateShooter() {
        if (AutoStates.launchstate == LaunchState.IDLE) {
            shooter.setTo(SHOOTER_IDLE_POWER);
        } else if (AutoStates.launchstate == LaunchState.END) {
            shooter.setTo(SHOOTER_END_POWER);
            turret.PIDto(0);
        }
    }

    /**
     * Autonomous Rotational Correction — tracks the Limelight target by
     * converting tx offset into a turret position adjustment via PID.
     */
    protected void ARC() {
        double turretPos = turret.getPos();

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
                turret.PIDto(targetPosition);
            }
        }
    }
}