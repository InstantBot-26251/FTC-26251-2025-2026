package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc.InertialLaunchCore;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeState;

// Complete shooting sequence: 1. Stop intake 2. Start ILC spinup (which internally handles transfer reverse + flywheel spinup) 3. Wait for ILC to be ready 4. Fire the shot (transfer forward) 5. Wait for shot duration 6. Stop shooting and return to idle
public class ShootSequenceCommand extends SequentialCommandGroup {

    private static final long SHOT_DURATION = 500; // Time to run transfer forward (tune if needed)

    public ShootSequenceCommand(InertialLaunchCore ilc, Intake intake) {
        addCommands(
                // Step 1: Stop intake
                new InstantCommand(() -> intake.setIntake(IntakeState.IDLE)),

                // Step 2: Start the spinup sequence (transfer reverse + flywheel spinup)
                new InstantCommand(ilc::startSpinup),

                // Step 3: Wait until ILC is ready (flywheel at target velocity)
                new WaitUntilCommand(ilc::isReady),

                // Step 4: Fire the shot (transfer forward)
                new InstantCommand(ilc::shoot),

                // Step 5: Wait for the shot to complete
                new WaitCommand(SHOT_DURATION),

                // Step 6: Stop shooting and return to idle
                new InstantCommand(ilc::stopShooting)
        );
    }
}