package org.firstinspires.ftc.teamcode.Requirements;

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
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class Rusty extends Robot {

    /// limelight
    private final Limelight3A limelight;
    public static double Ta;
    public static double Tx;


    /// Subsystems
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private GamepadEx driver;
    private IMU imu;

    /// class Variables
    private final OpMode opmode;
    private boolean isUnwrapping = false;
    private static final double TICKS_PER_REV = 2403;
    private static final double UnwindThreshold = 1400;
    DcMotor fL, bL, fR, bR;
    DcMotor[] motors;





    public Rusty(OpMode op) {
        opmode = op;
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void init(){
        initSubsystems();
        initControls();
        register(intake, transfer, shooter);
        limelight.start();
        schedule(new InstantCommand(() -> transfer.setPos(.85)), new InstantCommand(() -> shooter.moveServo(0)));
    }

    private void initSubsystems() {
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        shooter = new Shooter(opmode.hardwareMap);
        turret = new Turret(opmode.hardwareMap);


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
        turret.setPositionControl();
    }


    private void initControls(){
        driver = new GamepadEx(opmode.gamepad1);


        Button dpadUp = (new GamepadButton(driver, GamepadKeys.Button.DPAD_UP))
                .whenPressed(() -> shooter.HoodCMD.schedule());

        Button Start = (new GamepadButton(driver, GamepadKeys.Button.START))
                .whenPressed(() -> imu.resetYaw());

        Button rBumper = (new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(transfer.open)
                .whenHeld(intake.SpinIn.alongWith(transfer.SpinIn))
                .whenReleased(transfer.close.alongWith(transfer.StopTransfer));


        Button lBumper = (new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(transfer.open)
                .whenHeld(intake.SpinOut.alongWith(transfer.SpinOut))
                .whenReleased(new InstantCommand(()-> transfer.setPos(.85)).alongWith(new InstantCommand(transfer::PwrOff)));
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



        if (opmode.gamepad1.right_bumper) shooter.setTo(.5);

        else shooter.setTo(.4);


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

    public void TrackTag() {
        LLResult llresult = limelight.getLatestResult();
        double cPos = turret.getPos();
        if (Math.abs(cPos) >= UnwindThreshold) {
            if (isUnwrapping) return;

            turret.setPositionControl();
            isUnwrapping = true;
            double direction = Math.signum(cPos);
            int unwrapPos = (int) (cPos - (direction * TICKS_PER_REV));
            turret.goToPos(unwrapPos);
            turret.setVelocityControl();
        }
        isUnwrapping = false;
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();

            if (Math.abs(Tx) <= 2) turret.setVelocity(0);
            else {
                turret.setVelocityControl();
                double exponent = -.013 * (Math.abs(Tx) * 10 - 300);
                double t = 600 / (1 + Math.exp(exponent)) - 11.90418;
                double targetVel = Math.copySign(t, Tx);
                turret.setVelocity(targetVel * 150);
                if(Math.abs(targetVel) - Math.abs(turret.getVelo()) > 50){
                    turret.increaseFriction();
                }
            }
        }
    }
}
