package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TeleShootingCMD;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.constants.LaunchState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.TurretKalmanFilter;


public class Rusty extends Robot {



    private static final double SHOOTER_IDLE_VELOCITY = 1000;
    private static final double TRIGGER_DEADZONE = 0.05;
    private final OpMode opmode;
    private final double GoalX;
    private static final double GoalY = 144;

    // Subsystems
    private final Limelight3A limelight;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private Follower follower;
    private VoltageSensor voltageSensor;


    //Kalman Filter Vals
    private TurretKalmanFilter kalman;
    private static final double Q = 1;
    private static final double R_odom = 10;
    private static final double R_ll = 7;


    private static final double TX_FILTER_ALPHA = 0.4;
    private double filteredTx;

    // State
    public static LaunchState launchState;
    private int loopCounter = 0;

    public Rusty(OpMode op, Alliance a, AutoType at) {
        opmode = op;
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(a == Alliance.BLUE ? 0 : 1);
        GoalX = a == Alliance.BLUE ? 0 : 144;
        follower = Constants.createFollower(opmode.hardwareMap);

        Pose start = at == AutoType.CLOSE_15 ? new Pose(48, 110, Math.toRadians(225)) : new Pose(43, 17, Math.toRadians(180));

        start = a == Alliance.RED ? start.mirror(144) : start;

        follower.setPose(start);
    }

    public void init() {
        initSubsystems();
        initControls();

        launchState = LaunchState.IDLE;
        follower.startTeleopDrive();

        double initialTarget = turret.getPosDeg();

        kalman = new TurretKalmanFilter(Q, R_odom, R_ll, initialTarget);

        schedule(new InstantCommand(() -> shooter.moveServo(.8)));

        register(intake, transfer, shooter, turret);
    }

    private void initSubsystems() {
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        shooter = new Shooter(opmode.hardwareMap);
        turret = new Turret(opmode.hardwareMap);
        voltageSensor = opmode.hardwareMap.voltageSensor.iterator().next();
        limelight.start();
    }

    private void initControls() {
        GamepadEx driver = new GamepadEx(opmode.gamepad1);
        TeleShootingCMD shootingCMD = new TeleShootingCMD(shooter, transfer, intake, turret, limelight, voltageSensor);

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
            transfer.Spin(-.6);
        } else if (!opmode.gamepad1.right_bumper && !opmode.gamepad1.left_bumper) {
            intake.PwrOff();
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
