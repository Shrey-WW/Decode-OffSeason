package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.Requirements.Rusty;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp (name = "MainTeleOp", group = "teleop")
public class MainTeleOp extends CommandOpMode {
    Follower follower;
    Rusty rusty;

    Gamepad.LedEffect flashingColors = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 250)
            .addStep(0, 1, 0, 250)
            .addStep(0, 0, 1, 250)
            .setRepeating(true)
            .build();

    Gamepad.RumbleEffect rumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS)
            .build();

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rusty = new Rusty( this);
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();
        rusty.init();
        rusty.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        gamepad1.setLedColor(0.721568627, 0.450980392, 0.2, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.runLedEffect(flashingColors);
        gamepad2.runRumbleEffect(rumble);
    }

    @Override
    public void run(){
        rusty.run();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
    }
}