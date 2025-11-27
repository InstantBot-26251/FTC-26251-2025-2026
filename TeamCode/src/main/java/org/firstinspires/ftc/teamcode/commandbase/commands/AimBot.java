package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.globals.Enigma;

public class AimBot extends SequentialCommandGroup {
    public AimBot() {
        Enigma robot = Enigma.getInstance();
        addCommands(
                new AimCommand()
        );
    }
}
