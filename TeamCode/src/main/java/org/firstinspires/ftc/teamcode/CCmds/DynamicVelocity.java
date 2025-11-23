package org.firstinspires.ftc.teamcode.CCmds;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Flywheel;

public class DynamicVelocity extends CommandBase {


    private final Flywheel subsystem;

    public DynamicVelocity(Flywheel sub){
        subsystem = sub;

    }

    @Override
    public void initialize(){
        subsystem.setPwr(0);
    }

}
