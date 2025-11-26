package org.firstinspires.ftc.teamcode.Subsystems_Solvers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CCmds.TrackTag;

public class Rusty2 extends Robot {

    private final OpModeType optype;
    private final Limelight3A limelight;
    public static double Ta;
    /*  I_ - Intake
        F_ - Flywheel
        T_ - Turret
        Ts_ - Transfer
     */
    private Intake intake;
    private Transfer transfer;
    private Flywheel flywheel;
    private Turret turret;
    IMU imu;
    TrackTag T_Default;
    private final OpMode opmode;
    private Button start;

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
    }

    private void initSubsystems() {
        intake = new Intake(opmode.hardwareMap);
        transfer = new Transfer(opmode.hardwareMap);
        flywheel = new Flywheel(opmode.hardwareMap);
        turret = new Turret(opmode.hardwareMap);

        imu = opmode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }


    public void initBinds(){
        GamepadEx driver = new GamepadEx(opmode.gamepad1);

        T_Default = new TrackTag(turret);


        Button cross = (new GamepadButton(driver, GamepadKeys.Button.A))
                .whenPressed((transfer.transferCMD));


        Button triangle = (new GamepadButton(driver, GamepadKeys.Button.Y))
                .whenPressed(() -> flywheel.setTo(.7));
        Button circle = (new GamepadButton(driver, GamepadKeys.Button.B))
                .whenPressed(() -> flywheel.setTo(.5));
        Button square = (new GamepadButton(driver, GamepadKeys.Button.X))
                .whenPressed(() -> flywheel.setTo(.3));
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
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()){
            Ta = llresult.getTa();
            opmode.telemetry.addData("ta", Ta);
        }

        opmode.telemetry.addData("flywheel velo", flywheel.getVelo());
        opmode.telemetry.update();
        CommandScheduler.getInstance().run();
    }


}
