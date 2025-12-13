package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeState;

public class IntakeCommand extends CommandBase {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntake(IntakeState.INTAKING);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntake(IntakeState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}