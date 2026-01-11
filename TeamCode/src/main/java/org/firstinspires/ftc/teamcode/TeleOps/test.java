package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class test extends OpMode {
    Gamepad.LedEffect flashingColors = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 333)
            .addStep(0, 1, 0, 333)
            .addStep(0, 0, 1, 333)
            .setRepeating(true)
            .build();

    Gamepad.RumbleEffect rumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS)
            .build();

    @Override
    public void init(){
    }

    @Override
    public void loop(){
        gamepad1.setLedColor(0.9, 0.450980392, 0.2, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.runLedEffect(flashingColors);
        gamepad2.runRumbleEffect(rumble);
    }
}
