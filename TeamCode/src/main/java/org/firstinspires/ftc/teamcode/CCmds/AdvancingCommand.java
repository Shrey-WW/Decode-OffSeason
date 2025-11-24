package org.firstinspires.ftc.teamcode.CCmds;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Subsystems_Solvers.Transfer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.seattlesolvers.solverslib.command.Command;

public class AdvancingCommand extends CommandBase {;
    private final List<Command> cmdsList;
    private final int numCMDs;
    private int currentCMD;
    public AdvancingCommand(Command... cmds) {
        cmdsList = new ArrayList<>();
        Collections.addAll(cmdsList, cmds);
        numCMDs = cmdsList.size();
        currentCMD = -1;
    }

    public int getCurrentCMD(){
        return currentCMD;
    }

    @Override
    public void initialize() {
        currentCMD = (currentCMD + 1) % numCMDs;
        cmdsList.get(currentCMD).schedule();
    }

    @Override
    public boolean isFinished() {
        return cmdsList.get(currentCMD).isFinished();
    }
}
