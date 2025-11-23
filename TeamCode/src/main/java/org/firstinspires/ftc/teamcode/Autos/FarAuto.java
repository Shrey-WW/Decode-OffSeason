package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Old_Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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
    DcMotor fl, fr, bl, br;

    @Override
    public void onInit(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Old_Intake.X, TransferServo.X, Shooter.X)
        );
        TransferServo.X.open.schedule();
        bot = new Robot(this);
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void onStartButtonPressed(){
        AutoRoutine().schedule();
    }

    private Command AutoRoutine(){
        return new SequentialGroup(
                Shooter.X.setHood(.42),
                new InstantCommand(() -> Shooter.X.setPwr(.85)),
                new Delay(2.5),
                Old_Intake.X.SpinIn(1),
                new Delay(1.5),
                Old_Intake.X.SpinIn(.3),
                new Delay(2),
                Old_Intake.X.SpinIn(1),
                new Delay(5),
                Old_Intake.X.SpinOut(1),
                new Delay(1),
                TransferServo.X.close,
                new InstantCommand(() -> { Shooter.X.setPwr(0); Old_Intake.X.PwrOff(); }),
                new Delay(.5),
                new InstantCommand(() ->
                {
                    fl.setPower(.7);
                    fr.setPower(.7);
                    br.setPower(.7);
                    bl.setPower(.7);
                }),
                new Delay(.75),
                new InstantCommand(() ->
                {
                    fl.setPower(0);
                    fr.setPower(0);
                    br.setPower(0);
                    bl.setPower(0);
                })
        );
    }
    @Override
    public void onUpdate(){
        telemetry.addData("last velo", cVelo);
        telemetry.addData("current velo", Shooter.X.getVelo());
        telemetry.update();
    }
}
