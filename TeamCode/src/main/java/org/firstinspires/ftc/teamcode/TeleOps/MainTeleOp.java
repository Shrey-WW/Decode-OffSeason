package org.firstinspires.ftc.teamcode.TeleOps;

import static dev.nextftc.bindings.Bindings.button;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    Robot bot;
    Button rTrigger, lTrigger, x2, x, rBumper, lBumper, rBumper2, lBumper2, triangle, square, circle, dpadUp, dpadDown;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void onInit() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.X, TransferServo.X, Shooter.X, Turret.X)
        );
        bot = new Robot(this);
        initButtons();
        Turret.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed(){
        TransferServo.X.open.schedule();
        Shooter.X.HighHood.schedule();
        Turret.X.posPID();
        bot.drive.schedule();
        assignButtons();
    }

    @Override
    public void onUpdate(){
        String ServoStatus = "closed";
        if (TransferServo.X.transfer.getCurrentCommand() == TransferServo.X.open)
        {
            ServoStatus = "open";
        }
        if (timer.milliseconds() > 50) {
            telemetry.addData("Shooting velocity", Shooter.X.getVelo());
            telemetry.addData("Shooting power", Shooter.X.getPwr());
            telemetry.addData("Transfer Servo Status", ServoStatus);
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
        rBumper2 = button(() -> gamepad2.right_bumper);
        lBumper2 = button(() -> gamepad2.left_bumper);
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
        rBumper2.whenBecomesTrue(() -> Turret.X.TurnRight().schedule());
        lBumper2.whenBecomesTrue(() -> Turret.X.TurnLeft().schedule());
    }
}