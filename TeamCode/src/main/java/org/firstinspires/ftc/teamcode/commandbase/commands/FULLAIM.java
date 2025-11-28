package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.globals.Enigma;

public class FULLAIM extends SequentialCommandGroup {
    public FULLAIM() {
        Enigma robot = Enigma.getInstance();
        addCommands(
                new AimCommand()
        );
    }
}
