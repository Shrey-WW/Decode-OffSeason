package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class BlindCodeAuto extends NextFTCOpMode {

    Robot bot;
    Command AutoSequence, GoBackwards, shoot, spamTransfer, runaway;

    @Override
    public void onInit(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.X, TransferServo.X, Shooter.X)
        );
        bot = new Robot(this);
        bot.hoodservo.setPosition(.6);
        TransferServo.X.transfer.schedule();
        TransferServo.X.transfer.schedule();
    }

    @Override
    public void onStartButtonPressed(){
        MakeCommands();
        GoBackwards.then(shoot).then(spamTransfer).schedule();
    }

    public void MakeCommands(){
        GoBackwards = new InstantCommand(() -> {
            bot.motors[0].setPower(-.7);
            bot.motors[1].setPower(-.7);
            bot.motors[2].setPower(-.7);
            bot.motors[3].setPower(-.7);
        }).endAfter(1);

        shoot = new ParallelGroup(
                new InstantCommand(() -> {
                    bot.motors[0].setPower(0);
                    bot.motors[1].setPower(0);
                    bot.motors[2].setPower(0);
                    bot.motors[3].setPower(0);
                })
                ,
                Shooter.X.ShortPowerShot,
                new InstantCommand(Intake.X.SpinIn(1))
        ).endAfter(10);

        spamTransfer = new ParallelGroup(
                new InstantCommand(Intake.X.SpinIn(1)),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.75)
        );


    }
}
