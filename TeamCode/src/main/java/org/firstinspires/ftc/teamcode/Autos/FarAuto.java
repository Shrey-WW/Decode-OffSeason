package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous (group = "auto")
public class FarAuto extends NextFTCOpMode {
    private double cVelo;
    Robot bot;

    @Override
    public void onInit(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.X, TransferServo.X, Shooter.X, Turret.X)
        );
        TransferServo.X.open.schedule();
        TransferServo.X.open.schedule();
        TransferServo.X.open.schedule();
        Turret.X.PIDReset();
        bot = new Robot(this);
    }

    @Override
    public void onStartButtonPressed(){
        Turret.X.posPID();
        AutoRoutine().schedule();

    }

    private Command AutoRoutine(){
        return new SequentialGroup(
                new ParallelGroup(
                        TransferServo.X.open,
                        Shooter.X.setHood(.55),
                        Turret.X.TurnTo((2403.125/360) * 20)
                ),
                new InstantCommand(() -> Shooter.X.setPwr(.9)),
                new Delay(4.5),
                new InstantCommand(() -> cVelo = Shooter.X.getVelo()),
                Intake.X.SpinIn(1),
                new WaitUntil(() -> cVelo - Shooter.X.getVelo() > 100),
                Intake.X.SpinIn(.65),
                new Delay(3.5),
                new InstantCommand(() -> cVelo = Shooter.X.getVelo()),
                Intake.X.SpinIn(1),
                new WaitUntil(() -> cVelo - Shooter.X.getVelo() > 100),
                new Delay(3.5),
                TransferServo.X.close,
                new Delay(2),
                TransferServo.X.open,
                new Delay(1.5),
                TransferServo.X.close,
                new Delay(2),
                TransferServo.X.open,
                new Delay(1.5),
                TransferServo.X.close,
                new Delay(2),
                TransferServo.X.open,
                new Delay(1.5),
                TransferServo.X.close,
                new InstantCommand(() -> { Shooter.X.setPwr(0); Intake.X.PwrOff(); }),
                new InstantCommand(() -> {
                    bot.motors[0].setPower(.7);
                    bot.motors[1].setPower(.7);
                    bot.motors[2].setPower(.7);
                    bot.motors[3].setPower(.7);
                }),
                new Delay(2),
                new InstantCommand(() -> {
                    bot.motors[0].setPower(0);
                    bot.motors[1].setPower(0);
                    bot.motors[2].setPower(0);
                    bot.motors[3].setPower(0);
                })
        );
    }
}
