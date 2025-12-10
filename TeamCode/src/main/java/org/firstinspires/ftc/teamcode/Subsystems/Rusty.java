package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Rusty extends Robot {

    private final Limelight3A limelight;
    public static double Ta;
    public static double Tx;
    GamepadEx driver;
    /*  I_ - Intake
        F_ - Flywheel
        T_ - Turret
        Ts_ - Transfer
     */
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    IMU imu;
    private final OpMode opmode;

    DcMotor fL, bL, fR, bR;
    DcMotor[] motors;

    private boolean isUnwrapping = false;
    private static final double TICKS_PER_REV = 2403;
    private static final double UnwindThreshold = 1400;

    private int ballshot = 0;

    public Rusty(OpMode op) {
        opmode = op;
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void init(){
        initTele();
        limelight.start();
        schedule(transfer.transferCMD, shooter.HoodCMD);
    }


    public void initTele() {
        initSubsystems();
        initBinds();
        register(intake, transfer, shooter);
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
        turret.setVelocityControl();
    }


    public void initBinds(){
        driver = new GamepadEx(opmode.gamepad1);

        Button cross = (new GamepadButton(driver, GamepadKeys.Button.A))
                .whenPressed((transfer.transferCMD));
        Button dpadUp = (new GamepadButton(driver, GamepadKeys.Button.DPAD_UP))
                .whenPressed(() -> shooter.HoodCMD.schedule());
        Button Start = (new GamepadButton(driver, GamepadKeys.Button.START))
                .whenPressed(() -> imu.resetYaw());
        Button rBumper = (new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER)).whenPressed(
                () -> intake.Spin(1)).whenReleased((intake::PwrOff));
        Button lBumper = (new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)).whenPressed(
                () -> intake.Spin(-1)).whenReleased((intake::PwrOff));
    }

    @Override
    public void run() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        Drive();
        TrackTag();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()){
            Ta = llresult.getTa();
            Tx = llresult.getTx();
            opmode.telemetry.addData("ta", Ta);
        }
        opmode.telemetry.addData("flywheel velo", shooter.getVelo());
        opmode.telemetry.update();
        CommandScheduler.getInstance().run();
    }

    public void ballshot(){
        int currentvel = shooter.getVelocity();
        if ((lastvel - currentvel) > 100)
        {
            ballshot += 1;
        }
        int lasvel = shooter.getVelocity();
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
