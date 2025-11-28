package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Rusty;
import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Transfer;

@Config
@TeleOp
        (name = "newTele", group = "teleop")
public class SolversTeleOp extends CommandOpMode {
    Rusty rusty;
    public static double target;
    private GamepadEx m_driverOp;
    private InstantCommand s_pwr, f_pwr, o_pwr;
    private Button buttonA, buttonB, buttonX, buttonY, buttonstart;
    Transfer transfer;
    Shooter shooter;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rusty = new Rusty(Rusty.OpModeType.TELEOP, this);
        rusty.init();
//        m_driverOp = new GamepadEx(gamepad1);
//        transfer = new Transfer(hardwareMap);
//        flywheel = new Flywheel(hardwareMap);
//
//        buttonA = (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
//                .whenPressed((transfer.transferCMD));
//        buttonY = (new GamepadButton(m_driverOp, GamepadKeys.Button.Y))
//                .whenPressed(() -> {
//                    flywheel.setTo(.7);
//                    flywheel.setPIDs();
//                });
//        buttonB = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
//                .whenPressed(() -> {
//                    flywheel.setTo(.5);
//                    flywheel.setPIDs();
//                });
//        buttonX = (new GamepadButton(m_driverOp, GamepadKeys.Button.X))
//                .whenPressed(() -> {
//                    flywheel.setTo(.3);
//                    flywheel.setPIDs();
//                });
//
//        buttonstart = (new GamepadButton(m_driverOp, GamepadKeys.Button.START))
//                .whenPressed(() -> {
//                    flywheel.setTo(0);
//                    flywheel.setPIDs();
//                });
//
//        schedule(transfer.transferCMD);
    }

    @Override
    public void run(){
//        if (flywheel.kp != Flywheel.Kp || flywheel.kv != Flywheel.Kv) {
//            flywheel.setPIDs();
//        }

//        telemetry.addData("Velocity", flywheel.getVelo());
//        telemetry.addData("target", target);
//        telemetry.addData("kp", flywheel.kp);
//        telemetry.addData("kv", flywheel.kv);
//        telemetry.update();
        rusty.run();

//        double[] Gains = flywheel.getPIDs();
//        double[] ff = flywheel.getFF();
//        if (Gains[0] != Flywheel.Kp || Gains[2] != Flywheel.Kd || ff[1] != Flywheel.Kv) {
//            flywheel.setPIDs();
//        }
//
//        telemetry.addData("Velocity", flywheel.getVelo());
//        telemetry.update();
//        CommandScheduler.getInstance().run();
    }

}
