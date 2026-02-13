package org.firstinspires.ftc.teamcode.auto.base;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TeleShootingCMD;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.LaunchState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Rusty extends Robot {

    private static final double TURRET_LIMIT_CW = 4.5;
    private static final double TURRET_LIMIT_CCW = -3.3;
    private static final double TURRET_RESET_TOLERANCE = 0.3;
    private static final double TX_TO_TURRET_GAIN = 2.0;

    private static final double SHOOTER_IDLE_POWER = 0.35;
    private static final double TRIGGER_DEADZONE = 0.05;

    // Hardware
    private final Limelight3A limelight;
    private final OpMode opmode;

    // Subsystems
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private Follower follower;

    // State
    public static LaunchState launchState;
    private boolean resettingTurret = false;
    private int loopCounter = 0;

    public Rusty(OpMode op, Alliance a) {
        opmode = op;
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(a == Alliance.BLUE ? 0 : 1);
    }

    public void init() {
        initSubsystems();
        initControls();
        launchState = LaunchState.IDLE;
        follower = Constants.createFollower(opmode.hardwareMap);
        follower.setStartingPose(new Pose(19, 135, Math.toRadians(0)));
        follower.startTeleopDrive();
        schedule(new InstantCommand(() -> shooter.moveServo(.8))
                .alongWith(new InstantCommand(() -> transfer.setPos(0))));
        register(intake, transfer, shooter, turret);
    }

    private void initSubsystems() {
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        shooter = new Shooter(opmode.hardwareMap);
        turret = new Turret(opmode.hardwareMap);
        limelight.start();
    }

    private void initControls() {
        GamepadEx driver = new GamepadEx(opmode.gamepad1);
        TeleShootingCMD shootingCMD = new TeleShootingCMD(shooter, transfer, intake, turret, limelight);

        new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(shootingCMD);

        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(intake.SpinOut.alongWith(transfer.SpinOut))
                .whenReleased(new InstantCommand(transfer::PwrOff));

        new GamepadButton(driver, GamepadKeys.Button.A)
                .whenPressed(() -> resettingTurret = true);

        new GamepadButton(driver, GamepadKeys.Button.B)
                .whenPressed(() -> turret.resetPos());
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        double turretPos = turret.getPos();

        ARC(turretPos);
        updateIntake();
        updateShooter();
        updateDrive();

        if (loopCounter % 8 == 0) {
            opmode.telemetry.addData("flywheel velo", shooter.getVelo());
            opmode.telemetry.addData("turret heading deg", turret.getTurretHeadingDeg());
            opmode.telemetry.update();
        }
        loopCounter++;
    }

    /**
     * Autonomous Rotational Correction — tracks the Limelight target by
     * converting tx offset into a turret position adjustment via PID.
     */
    private void ARC(double turretPos) {
        // Hard safety cutoff regardless of mode
        if (turretPos >= TURRET_LIMIT_CW || turretPos <= TURRET_LIMIT_CCW) {
            turret.pwrOff();
            return;
        }

        if (resettingTurret) {
            turret.PIDto(0);
            if (Math.abs(turretPos) < TURRET_RESET_TOLERANCE) {
                resettingTurret = false;
                turret.pwrOff();
            }
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

    private void updateIntake() {
        if (opmode.gamepad1.right_trigger > TRIGGER_DEADZONE) {
            intake.Spin(1);
        } else if (!opmode.gamepad1.right_bumper && !opmode.gamepad1.left_bumper) {
            intake.PwrOff();
            transfer.PwrOff();
        }
    }

    private void updateShooter() {
        if (launchState == LaunchState.IDLE) {
            shooter.setTo(SHOOTER_IDLE_POWER);
        }
    }

    private void updateDrive() {
        follower.setTeleOpDrive(
                -opmode.gamepad1.left_stick_y,
                -opmode.gamepad1.left_stick_x,
                -opmode.gamepad1.right_stick_x,
                true
        );
    }
}
