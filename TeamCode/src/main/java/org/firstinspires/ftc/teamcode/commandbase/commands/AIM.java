package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.globals.Enigma;

public class AIM extends SequentialCommandGroup {
    public AIM() {
        Enigma robot = Enigma.getInstance();
        addCommands(
                new AimLaunch()
        );
    }
}
