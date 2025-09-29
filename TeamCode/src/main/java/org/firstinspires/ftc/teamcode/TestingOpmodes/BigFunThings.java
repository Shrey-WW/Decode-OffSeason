package org.firstinspires.ftc.teamcode.TestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.velSquidMotor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "bigfun")
public class BigFunThings extends NextFTCOpMode {
    private Command driverControlled;
    @Override
    public void onInit() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
        final MotorEx frontRightMotor = new MotorEx("fr");
        final MotorEx backLeftMotor = new MotorEx("bl").reversed();
        final MotorEx backRightMotor = new MotorEx("br");
        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().touchpadFinger1y(),
                Gamepads.gamepad1().touchpadFinger1x(),
                Gamepads.gamepad1().rightStickX()
        );
    }

    @Override
    public void onStartButtonPressed(){
        driverControlled.schedule();
    }
}