package org.firstinspires.ftc.teamcode.Subsystems_Solvers;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class Flywheel extends SubsystemBase {
    private final MotorGroup ShootingMotors;
    public Flywheel(final HardwareMap hw){
        ShootingMotors = new MotorGroup(new MotorEx(hw, "shooting2"), new MotorEx(hw, "shooter1"));
        ShootingMotors.setRunMode(Motor.RunMode.VelocityControl);
        ShootingMotors.setVeloCoefficients(0.00009, 0, 0);
        ShootingMotors.setFeedforwardCoefficients(0, .00044);
        ShootingMotors.set();
    }


}
