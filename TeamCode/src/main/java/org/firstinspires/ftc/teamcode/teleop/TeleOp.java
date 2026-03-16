package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Logan;

public class TeleOp extends OpMode {
    Logan logan;

    @Override
    public void init() {
        logan = new Logan(this);
    }

    @Override
    public void start(){
        logan.start();
    }

    @Override
    public void loop(){
        logan.run();
    }
}
