package org.firstinspires.ftc.teamcode.CCmds;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Turret;

public class TrackTag extends CommandBase {
    Turret turret;
    public TrackTag(Turret t){
        turret = t;
        addRequirements(t);
    }

    @Override
    public void initialize (){

    }


}
