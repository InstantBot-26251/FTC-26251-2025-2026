package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.KickerSubsystem;

public class IntakeCommand extends CommandBase {

    private final KickerSubsystem kicker;
    private long lastUpdateTime = 0;
    private static final long UPDATE_INTERVAL_MS = 100;

    public IntakeCommand(KickerSubsystem kicker) {
        this.kicker = kicker;
        addRequirements(kicker);
    }

    @Override
    public void initialize() {
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();

        if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
            kicker.updateArtifactTracking();
            lastUpdateTime = currentTime;
        }
    }

    @Override
    public boolean isFinished() {
        return kicker.isFull();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop intake motor (add in later)
    }
}

