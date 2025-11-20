package org.firstinspires.ftc.teamcode.TeleOps;

import static dev.nextftc.bindings.Bindings.button;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@TeleOp(group = "teleop", name = "Main TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    public static ElapsedTime timer = new ElapsedTime();

    Robot bot;
    Button rTrigger, lTrigger, x2, x, rBumper, lBumper, triangle, square, circle, dpadUp;

    private Limelight3A limelight;
    private IMU imu;
    private Command setVelPID;
    private static final double TICKS_PER_REV = 2403.125;
    private static final double unwrapThreshold = 2200;
    private static final double bearingMin = 1.5;

    private long lastLoop = System.nanoTime();
    private double lastHeading = 0;

    private boolean isUnwrapping = false;

    @Override
    public void onInit() {
        addComponents(
                new SubsystemComponent(Intake.X, TransferServo.X, Shooter.X, Turret.X),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
        bot = new Robot(this);
        initButtons();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        bot = new Robot(this);
    }

    @Override
    public void onWaitForStart() {
        Turret.X.resetPwr();
        Turret.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed(){
        Turret.X.velPID();
        limelight.start();
        TransferServo.X.transfer.getCommand().schedule();
        Shooter.X.SwitchHood.getCommand().schedule();
        bot.drive.schedule();
        setVelPID = new InstantCommand(Turret.X::velPID);
        assignButtons();
    }

    @Override
    public void onUpdate(){
        TrackTag();
        String ServoStatus = "closed";
        if (TransferServo.X.transfer.getCurrentCommand() == TransferServo.X.open)
        {
            ServoStatus = "open";
        }
        if (timer.milliseconds() > 100) {
            telemetry.addData("turret pos", Turret.X.getPos());
            telemetry.addData("Shooting velocity", Shooter.X.getVelo());
            telemetry.addData("Shooting power", Shooter.X.getPwr());
            telemetry.addData("Transfer Servo Status", ServoStatus);
            telemetry.addData("last heading", lastHeading);
            telemetry.update();
            timer.reset();
        }
    }

    private void initButtons(){
        rTrigger = button(() -> gamepad1.right_trigger > 0.1);
        lTrigger = button(() -> gamepad1.left_trigger > 0.1);
        x2 = button(() -> gamepad2.a);
        x = button(() -> gamepad1.a);
        triangle = button(() -> gamepad1.y);
        square = button(() -> gamepad1.x);
        rBumper = button(() -> gamepad1.right_bumper);
        lBumper = button(() -> gamepad1.left_bumper);
        circle = button(() -> gamepad1.b);
        dpadUp = button(() -> gamepad1.dpad_up);
    }

    private void assignButtons(){
        rTrigger.whenTrue(() -> Intake.X.SpinIn(gamepad1.right_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        lTrigger.whenTrue(() -> Intake.X.SpinOut(gamepad1.left_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        x2.whenBecomesTrue(() -> TransferServo.X.transfer.getCommand().schedule());
        triangle.whenBecomesTrue(() -> Shooter.X.FullPowerShot.schedule());
        square.whenBecomesTrue(() -> Shooter.X.ShortPowerShot.schedule());
        x.whenBecomesTrue(() -> Shooter.X.MinPower.schedule());
        rBumper.whenBecomesTrue(() -> Shooter.X.IncPower.schedule());
        lBumper.whenBecomesTrue(() -> Shooter.X.DecPower.schedule());
        dpadUp.whenBecomesTrue(() -> Shooter.X.SwitchHood.getCommand().schedule());
    }

    private void TrackTag() {
        double cPos = Turret.X.getPos();
        long currentTime = System.nanoTime();

        if (isUnwrapping) {
            if (Math.abs(cPos) < 500) {
                isUnwrapping = false;
            } else {
                lastLoop = System.nanoTime();
                lastHeading = imu.getRobotYawPitchRollAngles().getYaw();
                return;
            }
        }

        if (Math.abs(cPos) >= unwrapThreshold){
            isUnwrapping = true;
            Turret.X.posPID();
            double direction = Math.signum(cPos);
            double unwrapPos = cPos - (direction * TICKS_PER_REV);
            Turret.X.TurnTo(unwrapPos).schedule();
            setVelPID.afterTime(1.2).schedule();
            lastLoop = currentTime;
            lastHeading = imu.getRobotYawPitchRollAngles().getYaw();
            return;
        }

        double dt = (currentTime - lastLoop) / 1e9;
        if (dt <= 0 || dt > .2) dt = .02;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw();

        double dHeading = currentHeading - lastHeading;
        if (dHeading < -180) dHeading += 360;
        else if (dHeading > 180) dHeading -= 360;

        double rawHeadingVel = dHeading / dt;
        double headingFeedForward = rawHeadingVel * (TICKS_PER_REV / 360.0) * .6;

        double targetVel = 0;

        limelight.updateRobotOrientation(currentHeading);
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            if (Math.abs(Tx) > bearingMin){
                double val = .004 * Math.abs(Tx) * 10;
                double SigmoidVal = calculateSigmoid(val);
                double speed = 6.7 * 400 * SigmoidVal;
                targetVel = Math.copySign(speed, Tx);
            }

        }
        Turret.X.runTo((targetVel)).schedule();

        if (timer.milliseconds() > 100) {
            telemetry.addData("change in time", dt);
            telemetry.addData("Heading FF", rawHeadingVel);
            telemetry.addData("heading feedforward", headingFeedForward);
        }
        lastLoop = System.nanoTime();
        lastHeading = currentHeading;
    }

    private double calculateSigmoid(double input){
        return (input / (1.0 + Math.abs(input)));
    }
}