package org.firstinspires.ftc.teamcode.solverslib.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;

public class Shoot extends CommandBase {
    private final Robot robot;

    public Shoot() {
        robot = Robot.getInstance();

        addRequirements(robot.ilc, robot.intake);
    }

    @Override
    public void initialize() {
        // Stop Intake to prepare for shooting
        robot.intake.setIntake(IntakeState.STOP);
    }

    @Override
    public void execute() {
        if (robot.ilc.isReadyToShoot()) {
            robot.intake.setIntake(IntakeState.STOP);
        }
        else {
            robot.intake.setIntake(IntakeState.TRANSFER);
        }
    }

    @Override
    public void end(boolean interrupted) {
        robot.intake.setIntake(IntakeState.STOP);
    }


}
