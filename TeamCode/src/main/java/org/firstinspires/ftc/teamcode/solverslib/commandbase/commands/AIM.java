package org.firstinspires.ftc.teamcode.solverslib.commandbase.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

public class AIM extends SequentialCommandGroup {
    public AIM() {
        addCommands(
                new AimLaunch()
        );
    }
}
