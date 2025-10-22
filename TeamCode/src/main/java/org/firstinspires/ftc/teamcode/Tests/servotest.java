package org.firstinspires.ftc.teamcode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@TeleOp
public class servotest extends NextFTCOpMode {
    Button a, b;
    @Override
    public void onInit(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(TransferServo.X)
        );
        a = button(() -> gamepad1.a);
        b = button(() -> gamepad1.b);
    }

    @Override
    public void onStartButtonPressed(){
        a.whenBecomesTrue(TransferServo.X.open);
        b.whenBecomesTrue(TransferServo.X.close);
    }
    @Override
    public void onUpdate(){

    }
}
