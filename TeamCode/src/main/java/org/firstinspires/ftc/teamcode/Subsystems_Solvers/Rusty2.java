package org.firstinspires.ftc.teamcode.Subsystems_Solvers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.Trigger;
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
    OpMode opmode;
    GamepadEx driver;
    InstantCommand I_SpinIn, I_SpinOut, I_PwrOff;
    TriggerReader rTrigger, lTrigger;
    public enum OpModeType {
        TELEOP, AUTO
    }

    public Rusty2(OpModeType type, OpMode op) {
        opmode = op;
        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }


    public void initTele() {
        initSubsystems();
        register(intake);
        createBinds();
        initCMDs();
        assignBinds();
    }

    private void initAuto() {
        initPaths();
    }

    private void initPaths(){

    }

    private void createBinds(){
        driver = new GamepadEx(opmode.gamepad1);
        lTrigger = new TriggerReader(driver, GamepadKeys.Trigger.RIGHT_TRIGGER,.1);
        rTrigger = new TriggerReader(driver, GamepadKeys.Trigger.RIGHT_TRIGGER,.1);
    }

    private void assignBinds(){
        new Trigger(rTrigger::isDown).whenActive(I_SpinIn);
        new Trigger(lTrigger::isDown).whenActive(I_SpinOut);
    }

    private void initSubsystems(){
        intake = new Intake(opmode.hardwareMap);
    }

    private void initCMDs(){
        I_SpinIn = new InstantCommand(() -> intake.SpinIn(opmode.gamepad1.right_trigger));
        I_SpinOut = new InstantCommand(() -> intake.SpinOut(opmode.gamepad1.left_trigger));
        I_PwrOff = new InstantCommand(intake::PwrOff);
    }
}
