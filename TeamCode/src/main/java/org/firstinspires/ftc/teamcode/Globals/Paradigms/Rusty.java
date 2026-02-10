package org.firstinspires.ftc.teamcode.Globals.Paradigms;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Globals.States.LaunchState;
import org.firstinspires.ftc.teamcode.Globals.States.Alliance;
import org.firstinspires.ftc.teamcode.SolversLib.CMDs.TeleShootingCMD;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.SolversLib.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Rusty extends Robot {

    /// limelight
    private final Limelight3A limelight;


    /// Subsystems
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private IMU imu;
    private Follower follower;


    /// class Variables
    private double GoalX;
    private int GoalY;
    boolean Reseting = false;
    private final OpMode opmode;
    public static LaunchState launchState;
    private final Alliance alliance;
    DcMotor fL, bL, fR, bR;
    DcMotor[] motors;





    public Rusty(OpMode op, Alliance a) {
        opmode = op;
        alliance = a;
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        if (alliance == Alliance.BLUE) {
            limelight.pipelineSwitch(0);
            GoalX = 13;
            GoalY = 135;
        }
        else {
            limelight.pipelineSwitch(1);
            GoalX = 132.5;
            GoalY = 135;
        }
    }

    public void init(){
        initSubsystems();
        initControls();
        launchState = LaunchState.IDLE;
        follower = Constants.createFollower(opmode.hardwareMap);
        follower.setStartingPose(new Pose(19, 135, Math.toRadians(0)));
        follower.startTeleopDrive();
        schedule(new InstantCommand(() -> shooter.moveServo(.8)).alongWith(new InstantCommand(() -> transfer.setPos(0))));
        register(intake, transfer, shooter, turret);
    }

    private void initSubsystems() {
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        shooter = new Shooter(opmode.hardwareMap);
        turret = new Turret((opmode.hardwareMap));

        fL = opmode.hardwareMap.get(DcMotor.class, "fl");
        fR = opmode.hardwareMap.get(DcMotor.class, "fr");
        bL = opmode.hardwareMap.get(DcMotor.class, "bl");
        bR = opmode.hardwareMap.get(DcMotor.class, "br");
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = new DcMotor[]{fL, bL, fR, bR};

        imu = opmode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        limelight.start();
    }


    private void initControls() {
        GamepadEx driver = new GamepadEx(opmode.gamepad1);

        TeleShootingCMD shootingCMD = new TeleShootingCMD(shooter, transfer, intake, turret, limelight);

        Button rBumper = (new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER))
                .whenHeld(shootingCMD);

        Button lBumper = (new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER))
                .whenHeld(intake.SpinOut.alongWith(transfer.SpinOut))
                .whenReleased((new InstantCommand(transfer::PwrOff)));


        Button a = (new GamepadButton(driver, GamepadKeys.Button.A))
                .whenPressed(() -> Reseting = true);

        Button b = (new GamepadButton(driver, GamepadKeys.Button.B))
                .whenPressed(() -> turret.resetPos());
    }

    @Override
    public void run() {

        if (!Reseting) {
            ARC();
        }

        if (Reseting){
            turret.PIDto(0);
            if (Math.abs(0 - turret.getPos()) < .3){
                Reseting = false;
                turret.pwrOff();
            }
        }

        if (opmode.gamepad1.right_trigger > .05) {
            intake.Spin(1);
        }
        else if (opmode.gamepad1.right_trigger <= 0.3 && !opmode.gamepad1.right_bumper && !opmode.gamepad1.left_bumper) {
            intake.PwrOff();
            transfer.PwrOff();
        }

        if (launchState == LaunchState.IDLE) {
            shooter.setTo(.35);
        }

        follower.setTeleOpDrive(-opmode.gamepad1.left_stick_y, -opmode.gamepad1.left_stick_x, -opmode.gamepad1.right_stick_x, true);
        follower.update();

        opmode.telemetry.addData("flywheel velo", shooter.getVelo());
        opmode.telemetry.addData("turretheading deg", turret.getTurretHeadingDeg());
        opmode.telemetry.update();
        CommandScheduler.getInstance().run();
    }

    /**
     * Autonomous Rotational Correction!!!
     */
    public void ARC(){
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            double RadianTx = -Math.toRadians(Tx);
            double targetPosition = RadianTx * 2 + turret.getPos();
            if (targetPosition >= 4.5 || targetPosition <= -3.3) {
                turret.pwrOff();
            } else {
                turret.PIDto(targetPosition);
            }
        }

        if (turret.getPos() <= -2.7 && turret.getPos() >= 5){
            turret.pwrOff();
        }
    }
//        else {
//            Pose cPos = follower.getPose();
//            double CorrectedHeading = Math.atan((GoalY - cPos.getY())/(GoalX - cPos.getX()));
//            if (CorrectedHeading < 0){
//                CorrectedHeading = Math.PI - CorrectedHeading;
//            }
//            double TurretFieldHeading = turret.getTurretHeadingRad() + cPos.getHeading();
//            double turretHeadingError = TurretFieldHeading - CorrectedHeading;
//            double targetTurretRad = (turret.getTurretHeadingRad() - turretHeadingError);
//            double targetPosition = targetTurretRad * 2.6;
//            if (targetPosition >= 5 || targetPosition <= -2.7){
//                turret.pwrOff();

}


