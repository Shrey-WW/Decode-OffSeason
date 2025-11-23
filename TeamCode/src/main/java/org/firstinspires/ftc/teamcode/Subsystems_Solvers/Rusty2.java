package org.firstinspires.ftc.teamcode.Subsystems_Solvers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

public class Rusty2 extends Robot {
    /*  I_ - Intake
        F_ - Flywheel
        T_ - Turret
        Ts_ - Transfer
     */

    Intake intake;
    Transfer transfer;
    Flywheel shooter;

    OpMode opmode;

    GamepadEx driver;
    Button buttonX, Triangle;
    TriggerReader rTrigger, lTrigger;
    OpModeType optype;

    Command I_SpinIn, I_SpinOut, I_PwrOff, Ts_gate, F_HalfSpeed;

    public enum OpModeType {
        TELEOP, AUTO
    }

    public Rusty2(OpModeType type, OpMode op) {
        opmode = op;
        optype = type;
    }

    public void init(){
        if (optype == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }


    public void initTele() {
        initSubsystems();
        initCMDs();
        assignBinds();
        register(intake, transfer, shooter);
        schedule(Ts_gate);
    }

    private void initSubsystems(){
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        shooter = new Flywheel(opmode.hardwareMap);
    }

    private void initCMDs(){
        I_SpinIn = new InstantCommand(() -> intake.SpinIn(opmode.gamepad1.right_trigger));
        I_SpinOut = new InstantCommand(() -> intake.SpinOut(opmode.gamepad1.left_trigger));
        I_PwrOff = new InstantCommand(intake::PwrOff);
        F_HalfSpeed = shooter.runTo(.1);
        Ts_gate = transfer.transferCMD;
    }

    public void assignBinds(){
        driver = new GamepadEx(opmode.gamepad1);

        buttonX = (new GamepadButton(driver, GamepadKeys.Button.A))
                .whenPressed((transfer.transferCMD));

        Triangle = (new GamepadButton(driver, GamepadKeys.Button.Y))
                .whenPressed(F_HalfSpeed);
    }

    private void initAuto() {
        initPaths();
    }

    private void initPaths(){

    }

    public double getVelo(){
        return shooter.getVelo();
    }

}
