package org.firstinspires.ftc.teamcode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shootmotor;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class flywheelTest extends NextFTCOpMode {

    boolean lastButtonStateUp = false;
    boolean lastButtonStateDown = false;
    double pwr = 0;
    Gamepad.RumbleEffect Goodrumble, badrumble;
    Button rTrigger, lTrigger, triangle, square;

    @Override
    public void onInit(){
            addComponents(
                    BulkReadComponent.INSTANCE,
                    BindingsComponent.INSTANCE,
                    new SubsystemComponent(Shootmotor.X, Intake.X)
            );

        Goodrumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0, .5, 200).build();
        badrumble = new Gamepad.RumbleEffect.Builder()
                .addStep(.5, 0, 200).build();
        rTrigger = button(() -> gamepad1.right_trigger > 0.1);
        lTrigger = button(() -> gamepad1.left_trigger > 0.1);
        triangle = button(() -> gamepad1.y);
        square = button(() -> gamepad1.x);
    }
    @Override
    public void onStartButtonPressed(){
        rTrigger.whenTrue(() -> Intake.X.SpinIn(gamepad1.right_trigger).schedule()              )
                .whenBecomesFalse(Intake.X::PwrOff);
        lTrigger.whenTrue(() -> Intake.X.SpinOut(gamepad1.left_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
    }

    @Override
    public void onUpdate(){
        boolean currentButtonPressUp = gamepad1.dpad_up;
        boolean currentButtonPressDown = gamepad1.dpad_down;
        if (currentButtonPressUp && !lastButtonStateUp){
            pwr += .02;
        }
        if (currentButtonPressDown && !lastButtonStateDown){
            pwr -= .02;
        }
        lastButtonStateUp = currentButtonPressUp;
        lastButtonStateDown = currentButtonPressDown;
        Shootmotor.X.setPwr(pwr);
        telemetry.addData("velos", Shootmotor.X.getVelo2());
        telemetry.addData("Velo2", Shootmotor.X.getVelo());
        if (Shootmotor.X.getVelo() >= 1380){
            gamepad1.runRumbleEffect(Goodrumble);
        }
        else if (Shootmotor.X.getVelo() >= 1500){
            gamepad1.runRumbleEffect(badrumble);
        }

        telemetry.addData("pwr", pwr);
        telemetry.update();
    }
}
