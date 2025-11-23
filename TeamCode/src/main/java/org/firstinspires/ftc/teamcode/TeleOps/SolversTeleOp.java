package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Intake;
import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Rusty2;
import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Transfer;


@TeleOp
        (name = "newTele", group = "teleop")
public class SolversTeleOp extends CommandOpMode {

    private GamepadEx m_driverOp;
    private Intake intake;
    private InstantCommand SpinIn, PwrOff;
    private Button buttonA;
    private Trigger rTrigger;
    double rTriggerVal = 0;
    TriggerReader triggerReader;
    Transfer transfer;
    @Override
    public void initialize(){
        m_driverOp = new GamepadEx(gamepad1);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        SpinIn = new InstantCommand(()->intake.SpinIn(1));
        PwrOff = new InstantCommand(() -> intake.PwrOff());

        buttonA = (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
                .whenPressed((transfer.transferCMD));

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
                new InstantCommand(() -> intake.SpinIn(1)));


    }



}
