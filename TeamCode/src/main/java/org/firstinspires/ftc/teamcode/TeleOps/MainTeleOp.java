package org.firstinspires.ftc.teamcode.TeleOps;

import static dev.nextftc.bindings.Bindings.button;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    Gamepad.RumbleEffect GoodRumble, BadRumble;
    Robot bot;
    Button rTrigger, lTrigger, x, rBumper, lBumper, triangle, square;

    @Override
    public void onInit() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.X, TransferServo.X, Shooter.X, Turret.X)
        );
        bot = new Robot(this);
        initButtons();
    }

    @Override
    public void onStartButtonPressed(){
        bot.hoodservo.setPosition(.5);
        bot.drive.schedule();
        assignButtons();
    }

    @Override
    public void onUpdate(){

        if (Shooter.X.getVelo() >= 1500){
            gamepad1.runRumbleEffect(BadRumble);
        }
        else if (Shooter.X.getVelo() >= 1380){
            gamepad1.runRumbleEffect(GoodRumble);
        }

        telemetry.addData("Shooting velocity", Shooter.X.getVelo());
        telemetry.addData("Shooting power", Shooter.X.getPwr());
        telemetry.update();
    }

    private void initButtons(){
        rTrigger = button(() -> gamepad1.right_trigger > 0.1);
        lTrigger = button(() -> gamepad1.left_trigger > 0.1);
        x = button(() -> gamepad1.a);
        triangle = button(() -> gamepad1.y);
        square = button(() -> gamepad1.x);
        rBumper = button(() -> gamepad1.right_bumper);
        lBumper = button(() -> gamepad1.left_bumper);

        GoodRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(.2, .2, 1).build();
        BadRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(.5, 0, 1).build();
    }

    private void assignButtons(){
        rTrigger.whenTrue(() -> Intake.X.SpinIn(gamepad1.right_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        lTrigger.whenTrue(() -> Intake.X.SpinOut(gamepad1.left_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        x.whenBecomesTrue(() -> TransferServo.X.transfer.getCommand().schedule());
        triangle.whenBecomesTrue(() -> Shooter.X.FullPowerShot.schedule());
        square.whenBecomesTrue(() -> Shooter.X.ShortPowerShot.schedule());
        rBumper.whenBecomesTrue(() -> Shooter.X.IncPower.schedule());
        lBumper.whenBecomesTrue(() -> Shooter.X.DecPower.schedule());
    }
}