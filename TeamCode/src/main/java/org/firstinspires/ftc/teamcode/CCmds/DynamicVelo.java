package org.firstinspires.ftc.teamcode.CCmds;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Flywheel;


public class DynamicVelo extends CommandBase {

    public static double pwr;
    Flywheel flywheel;
    public DynamicVelo(Flywheel fly){
        flywheel = fly;
        addRequirements(fly);
    }

    @Override
    public void initialize() {
        flywheel.setTo(pwr);
    }
}
