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
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    Robot bot;
    Button rTrigger, lTrigger, x2, x, rBumper, lBumper, triangle, square, circle, dpadUp;
    ElapsedTime timer = new ElapsedTime();
    private Limelight3A limelight;
    private IMU imu;
    private Command setVelPID;
    private double lastHeading;

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

    public void TrackTag() {
        long startTime = System.nanoTime();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        double Ticks_Per_Revolution = 2403.125;
        double cPos = Turret.X.getPos();
        if (llresult != null && llresult.isValid()) {
            double Tx = llresult.getTx();
            if (Math.abs(Tx) > 1.5){
                double multi = 1;
                double k = .013;
                double target = 600 / (1 + Math.pow(Math.E, -k * (Math.abs(Tx) * 10 - 300))) - 11.90418;
                if (Tx < 0) {
                    multi = -1;
                }
                long currentTime = System.nanoTime();
                double TpsOffset = ((orientation.getYaw() - lastHeading) * Ticks_Per_Revolution / 360) * 1000000000/(currentTime - startTime);
                Turret.X.runTo(multi * ((target * 6.7) + TpsOffset)).schedule();

                if (timer.milliseconds() > 100){
                    telemetry.addData("sys time", currentTime - startTime);
                    telemetry.addData("TPS Offset", TpsOffset);
                }
            }
        }
        if (cPos >= 2200){
            Turret.X.posPID();
            double pos = cPos - ((int) (cPos / Ticks_Per_Revolution)) * Ticks_Per_Revolution;
            Turret.X.TurnTo(pos).schedule();
            setVelPID.afterTime(1.2).schedule();
        }
        else if (cPos <= -2200){
            Turret.X.posPID();
            double pos = cPos + ((int) Math.abs(cPos) / Ticks_Per_Revolution) * Ticks_Per_Revolution;
            Turret.X.TurnTo(pos).schedule();
            setVelPID.afterTime(1.2).schedule();
        }
        lastHeading = orientation.getYaw();
    }
}