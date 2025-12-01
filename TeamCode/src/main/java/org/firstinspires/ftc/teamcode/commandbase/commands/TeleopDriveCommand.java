package org.firstinspires.ftc.teamcode.commandbase.commands;


import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {
    private final Drive drive;
    private DoubleSupplier fwd, str, rot;

    public TeleopDriveCommand(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot) {
        this.drive = Drive.getInstance();
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setTeleOpDrive(fwd.getAsDouble(), str.getAsDouble(), rot.getAsDouble(), false);
    }
}