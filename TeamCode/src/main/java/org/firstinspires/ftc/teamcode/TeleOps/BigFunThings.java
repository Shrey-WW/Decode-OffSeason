package org.firstinspires.ftc.teamcode.TeleOps;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Main TeleOp")
public class BigFunThings extends NextFTCOpMode {
    Robot bot;
    Button rTrigger, lTrigger, a;
    @Override
    public void onInit() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.X, TransferServo.X)
        );
        bot = new Robot(this);
        rTrigger = button(() -> gamepad1.right_trigger > 0.1);
        lTrigger = button(() -> gamepad1.left_trigger > 0.1);
        a = button(() -> gamepad1.a);
    }

    @Override
    public void onStartButtonPressed(){
        bot.drive.schedule();
        rTrigger.whenTrue(() -> Intake.X.SpinIn(gamepad1.right_trigger))
                .whenBecomesFalse(Intake.X::PwrOff);
        lTrigger.whenTrue(() -> Intake.X.SpinOut(gamepad1.left_trigger))
                .whenBecomesFalse(Intake.X::PwrOff);
        a.whenBecomesTrue(TransferServo.X::transfer);
    }

    @Override
    public void onUpdate(){
    }
}