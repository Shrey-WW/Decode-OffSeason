package org.firstinspires.ftc.teamcode.TeleOps;

import static dev.nextftc.bindings.Bindings.button;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shootmotor;
import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;
import org.firstinspires.ftc.teamcode.Subsystems.TurretMotor;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    boolean lastButtonStateUp = false;
    boolean lastButtonStateDown = false;
    Servo hoodservo;
    Gamepad.RumbleEffect Goodrumble, badrumble;
    Robot bot;
    Command FullPwrShot, HalfPwrShot;
    Button rTrigger, lTrigger, x, PadUp, PadDown, triangle, square;
    double pwr = 0;

    @Override
    public void onInit() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.X, TransferServo.X, Shootmotor.X, TurretMotor.X)
        );
        bot = new Robot(this);
        initButtons();
        Goodrumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0, .5, 200).build();
        badrumble = new Gamepad.RumbleEffect.Builder()
                .addStep(.5, 0, 200).build();
        hoodservo = hardwareMap.get(Servo.class, "hood");
        FullPwrShot = new InstantCommand(() -> pwr = 1);
        HalfPwrShot = new InstantCommand(() -> pwr = .84);
    }

    @Override
    public void onStartButtonPressed(){
        hoodservo.setPosition(.5);
        bot.drive.schedule();
        assignButtons();
    }

    @Override
    public void onUpdate(){
        boolean currentButtonPressUp = gamepad1.right_bumper;
        boolean currentButtonPressDown = gamepad1.left_bumper;
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

        telemetry.addData("Shooting power", pwr);
        telemetry.update();

    }

    private void initButtons(){
        rTrigger = button(() -> gamepad1.right_trigger > 0.1);
        lTrigger = button(() -> gamepad1.left_trigger > 0.1);
        x = button(() -> gamepad1.a);
        PadUp = button(() -> gamepad1.dpad_up);
        PadDown = button(() -> gamepad1.dpad_down);
        triangle = button(() -> gamepad1.y);
        square = button(() -> gamepad1.x);
    }

    private void assignButtons(){
        rTrigger.whenTrue(() -> Intake.X.SpinIn(gamepad1.right_trigger).schedule()              )
                .whenBecomesFalse(Intake.X::PwrOff);
        lTrigger.whenTrue(() -> Intake.X.SpinOut(gamepad1.left_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        x.whenBecomesTrue(() -> TransferServo.X.transfer.getCommand().schedule());
        PadDown.whenBecomesTrue(() -> TurretMotor.X.SlowDown.schedule());
        PadUp.whenBecomesTrue(() -> TurretMotor.X.SpeedUp.schedule());
        triangle.whenBecomesTrue(() -> FullPwrShot.schedule());
        square.whenBecomesTrue(() -> HalfPwrShot.schedule());
    }
}