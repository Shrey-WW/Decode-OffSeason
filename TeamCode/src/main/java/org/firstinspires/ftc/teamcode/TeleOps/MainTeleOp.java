package org.firstinspires.ftc.teamcode.TeleOps;

import static dev.nextftc.bindings.Bindings.button;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    Gamepad.RumbleEffect GoodRumble, BadRumble;
    Robot bot;
    Button rTrigger, lTrigger, x, rBumper, lBumper, triangle, square, circle, dpadUp, dpadDown;
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
        if (timer.milliseconds() > 50) {
            telemetry.addData("Shooting velocity", Shooter.X.getVelo());
            telemetry.addData("Shooting power", Shooter.X.getPwr());
            telemetry.update();
            timer.reset();
        }
//        if (Shooter.X.getVelo() >= 1500){
//            gamepad1.runRumbleEffect(BadRumble);
//        }
//        else if (Shooter.X.getVelo() >= 1380){
//            gamepad1.runRumbleEffect(GoodRumble);
//        }
    }

    private void initButtons(){
        rTrigger = button(() -> gamepad1.right_trigger > 0.1);
        lTrigger = button(() -> gamepad1.left_trigger > 0.1);
        x = button(() -> gamepad1.a);
        triangle = button(() -> gamepad1.y);
        square = button(() -> gamepad1.x);
        rBumper = button(() -> gamepad1.right_bumper);
        lBumper = button(() -> gamepad1.left_bumper);
        circle = button(() -> gamepad1.b);
        dpadUp = button(() -> gamepad1.dpad_up);
        dpadDown = button(() -> gamepad1.dpad_down);

//        GoodRumble = new Gamepad.RumbleEffect.Builder()
//                .addStep(.2, .2, 1).build();
//        BadRumble = new Gamepad.RumbleEffect.Builder()
//                .addStep(.5, 0, 1).build();
    }

    private void assignButtons(){
        rTrigger.whenTrue(() -> Intake.X.SpinIn(gamepad1.right_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        lTrigger.whenTrue(() -> Intake.X.SpinOut(gamepad1.left_trigger).schedule())
                .whenBecomesFalse(Intake.X::PwrOff);
        x.whenBecomesTrue(() -> TransferServo.X.transfer.getCommand().schedule());
        triangle.whenBecomesTrue(() -> Shooter.X.FullPowerShot.schedule());
        square.whenBecomesTrue(() -> Shooter.X.ShortPowerShot.schedule());
        circle.whenBecomesTrue(() -> Shooter.X.MinPower.schedule());
        rBumper.whenBecomesTrue(() -> Shooter.X.IncPower.schedule());
        lBumper.whenBecomesTrue(() -> Shooter.X.DecPower.schedule());
        dpadUp.whenBecomesTrue(() -> Shooter.X.SwitchHood.getCommand().schedule());
        dpadDown.whenBecomesTrue(() -> Turret.X.TurnToGoal(bot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
    }
}