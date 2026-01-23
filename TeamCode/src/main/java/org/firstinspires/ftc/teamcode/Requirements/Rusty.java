package org.firstinspires.ftc.teamcode.Requirements;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CMDs.TeleShootingCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Rusty extends Robot {

    /// limelight
//    private final Limelight3A limelight;
    public static double Ta;
    public static double Tx;


    /// Subsystems
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private GamepadEx driver;
    private IMU imu;
    private Follower follower;

    /// class Variables
    private final OpMode opmode;
    private boolean isUnwrapping = false;
    private static final double TICKS_PER_REV = 2403;
    private static final double UnwindThreshold = 1400;
    public static LaunchState launchState;
    DcMotor fL, bL, fR, bR;
    DcMotor[] motors;





    public Rusty(OpMode op) {
        opmode = op;
//        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        // limelight.pipelineSwitch(0);
    }

    public void init(){
        initSubsystems();
        initControls();
        register(intake, transfer, shooter);
        // limelight.start();
        schedule(new InstantCommand(() -> transfer.setPos(.2)));
        launchState = LaunchState.IDLE;
        follower = Constants.createFollower(opmode.hardwareMap);
        follower.startTeleopDrive();
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
    }


    private void initControls(){
        driver = new GamepadEx(opmode.gamepad1);

        TeleShootingCMD shootingCMD = new TeleShootingCMD(shooter, transfer, intake, 980);

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
//        Drive();

//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw());
//        LLResult llresult = limelight.getLatestResult();
//
//        if (llresult != null && llresult.isValid()){
//            Ta = llresult.getTa();
//            Tx = llresult.getTx();
//            opmode.telemetry.addData("ta", Ta);
//        }

        if (opmode.gamepad1.right_trigger > .05) {
            intake.Spin(1);
        }

        else if (opmode.gamepad1.right_trigger <= 0.3 && !opmode.gamepad1.right_bumper && !opmode.gamepad1.left_bumper) {
            intake.PwrOff();
            transfer.PwrOff();
        }

        if (launchState == LaunchState.SHOOTING){
            shooter.setTo(.44);
        }
        else if (launchState == LaunchState.IDLE){
            shooter.setTo(.35);
        }


        follower.setTeleOpDrive(-opmode.gamepad1.left_stick_y, -opmode.gamepad1.left_stick_x, -opmode.gamepad1.right_stick_x, true);
        follower.update();
        opmode.telemetry.addData("flywheel velo", shooter.getVelo());
        opmode.telemetry.update();
        CommandScheduler.getInstance().run();
    }



    public void Drive(){

        double y = driver.getLeftY(); // Remember, Y stick value is reversed
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motors[0].setPower(frontLeftPower);
        motors[1].setPower(backLeftPower);
        motors[2].setPower(frontRightPower);
        motors[3].setPower(backRightPower);
    }
}
