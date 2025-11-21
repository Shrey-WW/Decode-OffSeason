package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake implements SubsystemBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public static final Intake X = new Intake();
    private final MotorEx motor = new MotorEx("intake");
    
    public Intake(final HardwareMap hw, double pwr) {
        X = hw;
        Command SpinOut = new InstantCommand(() -> X.setPower(pwr));
        Command SpinIn = new InstantCommand(() -> X.setPower(-pwr));
        intakeCMD = new AdvancingCommand(SpinOut, SpinIn);
  }
}