package org.firstinspires.ftc.teamcode.Subsystems_Solvers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CCmds.DynamicVelo;

public class Rusty2 extends Robot {

    private OpModeType optype;
    private Limelight3A limelight;
    private double lastPwr = 0;
    /*  I_ - Intake
        F_ - Flywheel
        T_ - Turret
        Ts_ - Transfer
     */
    private Intake intake;
    private Transfer transfer;
    private Flywheel flywheel;
    DynamicVelo F_default;
    private final OpMode opmode;
    private GamepadEx driver;
    private Button cross, circle, square, triangle, start;

    public enum OpModeType {
        TELEOP, AUTO
    }

    public Rusty2(OpModeType type, OpMode op) {
        opmode = op;
        optype = type;
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void init(){
        if (optype == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
        limelight.start();
//        flywheel.setDefaultCommand(F_DefaultCMD);
    }


    public void initTele() {
        initSubsystems();
        initBinds();
        register(intake, transfer, flywheel);
        schedule(transfer.transferCMD);
        flywheel.setDefaultCommand(F_default);
    }

    private void initSubsystems() {
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        flywheel = new Flywheel(opmode.hardwareMap);
    }


    public void initBinds(){
        driver = new GamepadEx(opmode.gamepad1);

        F_default = new DynamicVelo(flywheel);


        cross = (new GamepadButton(driver, GamepadKeys.Button.A))
                .whenPressed((transfer.transferCMD));


        triangle = (new GamepadButton(driver, GamepadKeys.Button.Y))
                .whenPressed(() -> {
                    flywheel.setTo(.7);
                });
        circle = (new GamepadButton(driver, GamepadKeys.Button.B))
                .whenPressed(() -> {
                    flywheel.setTo(.5);
                });
        square = (new GamepadButton(driver, GamepadKeys.Button.X))
                .whenPressed(() -> {
                    flywheel.setTo(.3);
                });
    }

    private void initAuto() {
        limelight.start();
        initPaths();
    }

    private void initPaths(){

    }

    public double getVelo(){
        return flywheel.getVelo();
    }

    @Override
    public void run() {
        flywheel.setTo(updateVeloPwr());
        LLResult llresult = limelight.getLatestResult();
        if (llresult.isValid() && llresult != null){
            opmode.telemetry.addData("ta", llresult.getTa());
        }

        opmode.telemetry.addData("pwr", updateVeloPwr());
        opmode.telemetry.update();
        CommandScheduler.getInstance().run();
    }

    public double updateVeloPwr(){
        LLResult llresult = limelight.getLatestResult();
        if (llresult.isValid() && llresult != null){
            double Ta = llresult.getTa();
            double velo = 1470 * Math.pow(Ta, -.141712);
            double pwr = 0.2057298 * Math.pow(1.000755, velo);
            lastPwr = pwr;
            return pwr;
        }
        return lastPwr;
    }


}
