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

@TeleOp(name = "bigfun")
public class BigFunThings extends NextFTCOpMode {
    Robot bot;
    Button rTrigger, lTrigger, x, circle, square;
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
        x = button(() -> gamepad1.a);
        circle = button(() -> gamepad1.b);
        square = button(() -> gamepad1.x);
    }

    @Override
    public void onStartButtonPressed(){
        bot.drive.schedule();
        rTrigger.whenTrue(() -> Intake.X.SpinIn(gamepad1.right_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        lTrigger.whenTrue(() -> Intake.X.SpinOut(gamepad1.left_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        x.whenBecomesTrue(TransferServo.X.open);
        circle.whenBecomesTrue(TransferServo.X.close);
        square.whenBecomesTrue(TransferServo.X.switchCMD);
    }

    @Override
    public void onUpdate(){
    }
}