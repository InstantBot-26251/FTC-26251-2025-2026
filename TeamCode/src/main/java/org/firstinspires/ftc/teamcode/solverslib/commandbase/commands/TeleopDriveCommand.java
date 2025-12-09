package org.firstinspires.ftc.teamcode.solverslib.commandbase.commands;


import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {
//    private final Drive drive;
    private final Robot robot;
    private DoubleSupplier fwd, str, rot;

    public TeleopDriveCommand(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot) {
        robot = Robot.getInstance();
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;

        addRequirements(robot.drive);
    }

    @Override
    public void execute() {
        robot.drive.setTeleOpDrive(fwd.getAsDouble(), str.getAsDouble(), rot.getAsDouble(), false);
    }
}