package org.firstinspires.ftc.teamcode.ArchiveFiles.CCmds;

import java.util.Collections;
import java.util.List;

import java.util.ArrayList;

import dev.nextftc.core.commands.Command;

public class SwitchCMD extends Command {
    private final List<Command> cmdsList;
    private final int numCMDs;
    private int currentCMD;

    public SwitchCMD(Command... Cmds) {
        cmdsList = new ArrayList<>();
        Collections.addAll(cmdsList, Cmds);
        numCMDs = cmdsList.size();
        currentCMD = -1;
    }

    public Command getCommand() {
        currentCMD = (currentCMD + 1) % numCMDs;
        return cmdsList.get(currentCMD);
    }

    public Command getCurrentCommand(){
        int temp = currentCMD;
        if (temp == -1){
            temp = 0;
        }
        return cmdsList.get(temp);
    }


    @Override
    public boolean isDone() {
        return true;
    }

}
