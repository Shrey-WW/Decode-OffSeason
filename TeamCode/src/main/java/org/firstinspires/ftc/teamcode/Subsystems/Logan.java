package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.InterpLUT;
import org.firstinspires.ftc.teamcode.util.TurretKalmanFilter;

public class Logan {

    // Subsystems
    public final Intake intake;
    public final Transfer transfer;
    public final Shooter shooter;
    public final Turret turret;
    public final Follower follower;
    public final Limelight3A limelight;
    public final VoltageSensor voltageSensor;
    private final OpMode opmode;

    // Button states
    private boolean lastRB = false;
    private boolean lastRT = false;
    private boolean lastLT = false;

    // ARC (April-Tag Rotational Correction)
    private final TurretKalmanFilter kalman;
    private static final double Q = 1;
    private static final double R_odom = 10;
    private static final double R_ll = 7;
    private static final double TX_FILTER_ALPHA = 0.4;
    private double filteredTx;
    private final double GoalX, GoalY;

    protected final InterpLUT VELO = new InterpLUT();
    private static final double VOLTAGE_BASE = 12.5;
    private static final double VOLTAGE_MIN = 10.5;
    private static final double HEIGHT_LIMELIGHT = 16.5;
    private static final double LIMELIGHT_MOUNT_ANGLE = 12.68;
    private static final double HEIGHT_OF_APRILTAG = 29.5;
    private static final double DefaultTargetVel = 1200;
    private static final double DISTANCE_FILTER_ALPHA = 0.20;
    private double filteredDistance = Double.NaN;

    public Logan(OpMode opmode) {
        this.opmode = opmode;
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        shooter = new Shooter(opmode.hardwareMap);
        turret = new Turret(opmode.hardwareMap);
        voltageSensor = opmode.hardwareMap.voltageSensor.iterator().next();
        follower = Constants.createFollower(opmode.hardwareMap);
        limelight = opmode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        kalman = new TurretKalmanFilter(Q, R_odom, R_ll, 0);
        GoalX = 140;
        GoalY = 140;

        VELO.add(0, 800);
        VELO.add(24.4, 1050);
        VELO.add(41, 1140);
        VELO.add(66, 1310);
        VELO.add(76, 1370);
        VELO.add(90, 1640);
        VELO.add(100, 1800);
        VELO.add(120, 1860);
        VELO.createLUT();
    }

    public void start() {
        transfer.Reverse();
        intake.Off();
        shooter.idle();
        turret.track();
        follower.startTeleOpDrive();
        limelight.start();
    }

    public void runAuto(){
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult llResult = limelight.getLatestResult();
        ARC(llResult);
        VELO(llResult);
        intake.update();
        transfer.update();
        shooter.update();
        turret.update();
        follower.update();
    }

    public void runTeleOp() {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult llResult = limelight.getLatestResult();
        ARC(llResult);
        intake.update();
        transfer.update();
        shooter.update();
        turret.update();
        checkButtons(llResult);

        follower.setTeleOpDrive(
                -opmode.gamepad1.left_stick_y,
                -opmode.gamepad1.left_stick_x,
                -opmode.gamepad1.right_stick_x,
                true
        );
        follower.update();
    }

    private void checkButtons(LLResult llResult) {
        boolean rb = opmode.gamepad1.right_bumper;
        boolean rt = opmode.gamepad1.right_trigger > 0.1;
        boolean lt = opmode.gamepad1.left_trigger > 0.1;

        // Shooter
        if (!lastRB && rb) {
            shooter.shoot();
        } else if (lastRB && rb) {
            VELO(llResult);
        } else if (lastRB) {
            shooter.idle();
            transfer.Reverse();
        }
        lastRB = rb;

        // Intake
        if (!lastRT && rt) {
            intake.Intaking();
        } else if (!lastLT && lt) {
            intake.Reverse();
        } else if ((lastRT && !rt) || (lastLT && !lt)) {
            intake.Off();
        }

        lastRT = rt;
        lastLT = lt;
    }

    /**
     * April-Tag Rotational Correction
     */
    public void ARC(LLResult llResult) {
        Pose cPos = follower.getPose();
        double odomTarget = Math.toDegrees(angleWrap(
                Math.atan2(GoalY - cPos.getY(), GoalX - cPos.getX()) - cPos.getHeading()
        ));

        Double llTarget = null;
        if (llResult != null && llResult.isValid()) {
            filteredTx = TX_FILTER_ALPHA * llResult.getTx() + (1 - TX_FILTER_ALPHA) * filteredTx;
            llTarget = -filteredTx + turret.getDegreePosition();
        }

        turret.setTargetPosition(kalman.estimate(odomTarget, llTarget));
    }

    public void VELO(LLResult llResult) {
        double currentVoltage = Math.max(voltageSensor.getVoltage(), VOLTAGE_MIN);

        double targetVel = (llResult != null && llResult.isValid())
                ? VELO.get(getDistanceFromTag(llResult))
                : DefaultTargetVel;

        shooter.setVelocity(targetVel);

        if (shooter.getVelocity() > targetVel - getRecoveryOffset(targetVel, currentVoltage)) {
            transfer.Transfer();
            intake.Intaking();
        } else {
            transfer.Off();
            intake.RestIntake();
        }
    }

    private double getDistanceFromTag(@NonNull LLResult lr) {
        double raw = (HEIGHT_OF_APRILTAG - HEIGHT_LIMELIGHT) /
                (Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + lr.getTy())) * Math.cos(Math.toRadians(filteredTx)));

        filteredDistance = Double.isNaN(filteredDistance)
                ? raw
                : DISTANCE_FILTER_ALPHA * raw + (1 - DISTANCE_FILTER_ALPHA) * filteredDistance;

        if (filteredDistance >= 85) shooter.setServoPosition(.7);

        return filteredDistance;
    }

    private double getRecoveryOffset(double tVel, double cVoltage) {
        return (-0.16 * tVel + 340) * (cVoltage / VOLTAGE_BASE);
    }

    private double angleWrap(double radians) {
        if (radians > Math.PI)  radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}