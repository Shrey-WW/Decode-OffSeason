package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TeleShootingCMD;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.LaunchState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.TurretKalmanFilter;

import java.util.List;

public class Rusty extends Robot {

    private static final double TX_FILTER_ALPHA = 0.4;
    private double filteredTx;

    private static final double SHOOTER_IDLE_VELOCITY = 800;
    private static final double TRIGGER_DEADZONE = 0.05;

    private final OpMode opmode;
    private final double GoalX;
    private static final double GoalY = 144;
    public static double Q = 1;
    public static double R_odom = 10;
    public static double R_ll = 7;

    // Subsystems
    private TurretKalmanFilter kalman;
    private final Limelight3A limelight;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private Follower follower;
    private List<LynxModule> Hubs;

    // State
    public static LaunchState launchState;
    private int loopCounter = 0;

    public Rusty(OpMode op, Alliance a) {
        opmode = op;
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(a == Alliance.BLUE ? 0 : 1);
        follower = Constants.createFollower(opmode.hardwareMap);
        if (a == Alliance.BLUE) {
            follower.setStartingPose(new Pose(48, 110, Math.toRadians(225)));
            GoalX = 0;
        } else {
            GoalX = 144;
            follower.setStartingPose(new Pose(48, 110, Math.toRadians(225)).mirror(144));
        }
    }

    public void init() {
        initSubsystems();
        initControls();

        launchState = LaunchState.IDLE;
        follower.startTeleopDrive();
        Hubs = opmode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : Hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        double initialTarget = Math.toDegrees(angleWrap(
                Math.atan2(GoalY - 115, GoalX - 50) - follower.getHeading()
        ));

        kalman = new TurretKalmanFilter(Q, R_odom, R_ll, initialTarget);
        turret.resetEncoder();

        schedule(new InstantCommand(() -> shooter.moveServo(.8)));

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
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));

        for (LynxModule hub : Hubs) {
            hub.clearBulkCache();
        }

        ARC();
        updateIntake();
        updateShooter();
        updateDrive();

        if (loopCounter % 8 == 0) {
            opmode.telemetry.addData("flywheel velo", shooter.getVelo());
            opmode.telemetry.addData("turret heading deg", turret.getPosDeg());
            opmode.telemetry.update();
        }
        loopCounter++;
    }

    /**
     * April-Tag Rotational Correction
     */
    private void ARC() {
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

    private void updateIntake() {
        if (opmode.gamepad1.right_trigger > TRIGGER_DEADZONE) {
            intake.Spin(1);
        } else if (!opmode.gamepad1.right_bumper && !opmode.gamepad1.left_bumper) {
            intake.PwrOff();
            transfer.Spin(-.5);
        }
    }

    private void updateShooter() {
        if (launchState == LaunchState.IDLE) {
            shooter.setVelocity(SHOOTER_IDLE_VELOCITY);
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

    private double angleWrap(double radians) {
        if (radians > Math.PI) radians -= 2 * Math.PI;
        if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
