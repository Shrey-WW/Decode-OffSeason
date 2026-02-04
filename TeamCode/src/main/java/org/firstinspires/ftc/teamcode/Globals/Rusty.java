package org.firstinspires.ftc.teamcode.Globals;

import androidx.appcompat.widget.AlertDialogLayout;

import com.pedropathing.follower.Follower;
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
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CMDs.TeleShootingCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Rusty extends Robot {

    /// limelight
    private final Limelight3A limelight;
    private double Ta;
    private double Tx;


    /// Subsystems
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private GamepadEx driver;
    private IMU imu;
    private Follower follower;

    /// class Variables
    private InterpLUT veloLUT = new InterpLUT();
    private final OpMode opmode;
    private boolean isUnwrapping = false;
    private static final double TICKS_PER_REV = 2403;
    private static final double UnwindThreshold = 1400;
    private static double pwr;
    public static LaunchState launchState;
    private final Alliance alliance;
    DcMotor fL, bL, fR, bR;
    DcMotor[] motors;





    public Rusty(OpMode op, Alliance a) {
        opmode = op;
        alliance = a;
        pwr = 0;
        Ta = 0;
        veloLUT.add(.18, .67);
        veloLUT.add(.2, .6);
        veloLUT.add(.305, .59);
        veloLUT.add(.45, .5075);
        veloLUT.add(.69, .475);
        veloLUT.add(1.05, .4467);
        veloLUT.add(1.61,.43);
        veloLUT.add(2.81, .424);
        veloLUT.add(5.5, .403);
        veloLUT.createLUT();
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        if (alliance == Alliance.BLUE) { limelight.pipelineSwitch(0); }
        else { limelight.pipelineSwitch(1); }
    }

    public void init(){
        initSubsystems();
        initControls();
        launchState = LaunchState.IDLE;
        follower = Constants.createFollower(opmode.hardwareMap);
        follower.startTeleopDrive();
        schedule(new InstantCommand(() -> transfer.setPos(.2)), new InstantCommand(() -> shooter.moveServo(.8)));
        register(intake, transfer, shooter);
    }

    private void initSubsystems() {
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        shooter = new Shooter(opmode.hardwareMap);

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


    private void initControls(){
        driver = new GamepadEx(opmode.gamepad1);

        TeleShootingCMD shootingCMD = new TeleShootingCMD(shooter, transfer, intake, limelight, veloLUT);

        Button rBumper = (new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER))
                .whenHeld(shootingCMD);

        Button start = (new GamepadButton(driver, GamepadKeys.Button.START))
                .whenHeld(new InstantCommand(() -> follower.setHeading(0)));

        Button lBumper = (new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(transfer.open)
                .whenHeld(intake.SpinOut.alongWith(transfer.SpinOut))
                .whenReleased(new InstantCommand(()-> transfer.setPos(.2)).alongWith(new InstantCommand(transfer::PwrOff)));
    }

    @Override
    public void run() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()){
            Ta = llresult.getTa();
            Tx = llresult.getTx();
            opmode.telemetry.addData("ta", Ta);
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
        opmode.telemetry.update();
        CommandScheduler.getInstance().run();
    }

}
