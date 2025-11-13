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

@Autonomous (group = "auto")
public class FarAuto extends NextFTCOpMode {

    Robot bot;
    Command shoot, spamTransfer;

    @Override
    public void onInit(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.X, TransferServo.X, Shooter.X, Turret.X)
        );
        MakeCommands();
        Turret.X.PIDReset();
    }

    @Override
    public void onStartButtonPressed(){
        new SequentialGroup(
                new ParallelGroup(
                        TransferServo.X.open,
                        Shooter.X.HighHood
                ),
                shoot,
                spamTransfer
        );
    }

    public void MakeCommands(){
        shoot = new ParallelGroup(
                Shooter.X.FullPowerShot,
                new InstantCommand(Intake.X.SpinIn(1)).afterTime(2.5)
        ).endAfter(6.7);

        spamTransfer = new SequentialGroup(
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.75),
                TransferServo.X.transfer.afterTime(.5),
                TransferServo.X.transfer.afterTime(.75)
        );


    }
}
