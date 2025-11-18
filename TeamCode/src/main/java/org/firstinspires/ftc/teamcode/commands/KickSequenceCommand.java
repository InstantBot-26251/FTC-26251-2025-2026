package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.subsystems.TRIkicker.TRIConstants.*;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.enums.OperatingMode;

public class KickSequenceCommand extends CommandBase {

    private final KickerSubsystem kicker;
    private int currentSlot = 0;
    private long kickStartTime = 0;
    private long delayStartTime = 0;
    private boolean isKicking = false;
    private boolean isDelaying = false;
    private boolean[] shouldKick = new boolean[3];

    public KickSequenceCommand(KickerSubsystem kicker) {
        this.kicker = kicker;
        addRequirements(kicker);
    }

    @Override
    public void initialize() {
        currentSlot = 0;
        isKicking = false;
        isDelaying = false;

        // Determine which slots should kick based on mode
        if (kicker.getMode() == OperatingMode.TELEOP) {
            // In teleop, kick all slots regardless
            shouldKick[0] = true;
            shouldKick[1] = true;
            shouldKick[2] = true;
            Log.i("KickSequence", "TELEOP mode - kicking all slots");
        } else {
            // In auto/endgame, only kick slots that match the pattern
            shouldKick[0] = kicker.slotMatchesTarget(0);
            shouldKick[1] = kicker.slotMatchesTarget(1);
            shouldKick[2] = kicker.slotMatchesTarget(2);
            Log.i("KickSequence", "AUTO mode - kicking matching slots: " +
                    shouldKick[0] + ", " + shouldKick[1] + ", " + shouldKick[2]);
        }
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();

        if (currentSlot >= 3) {
            return; // Done with all slots
        }

        if (isKicking) {
            // Wait for kick duration to complete
            if (currentTime - kickStartTime >= KICK_DURATION * 1000) {
                kicker.resetSlot(currentSlot);

                // Clear tracking for this slot since we kicked it out
                kicker.clearSlotTracking(currentSlot);

                isKicking = false;

                // Start delay before next kick
                if (currentSlot < 2) {
                    isDelaying = true;
                    delayStartTime = currentTime;
                } else {
                    currentSlot++; // Move to done state
                }
            }
        } else if (isDelaying) {
            // Wait for inter-kick delay
            if (currentTime - delayStartTime >= INTER_KICK_DELAY * 1000) {
                isDelaying = false;
                currentSlot++;
            }
        } else {
            // Start kicking current slot if it should kick
            if (shouldKick[currentSlot]) {
                kicker.kickSlot(currentSlot);
                isKicking = true;
                kickStartTime = currentTime;
            } else {
                // Skip this slot, move to delay or next slot
                if (currentSlot < 2) {
                    isDelaying = true;
                    delayStartTime = currentTime;
                } else {
                    currentSlot++;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return currentSlot >= 3;
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure all kickers are reset
        kicker.resetAllSlots();
        Log.i("KickSequence", "Sequence complete");
    }
}