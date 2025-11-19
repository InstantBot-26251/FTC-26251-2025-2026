package org.firstinspires.ftc.teamcode.subsystems.TRIkicker;

import android.util.Log;

import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.enums.ArtifactColor;
import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.enums.OperatingMode;

import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;

import java.util.HashMap;
import java.util.Map;

public class KickerSubsystem extends SubsystemTemplate {
    private Slot[] slots = new Slot[3];
    private OperatingMode currentMode = OperatingMode.TELEOP;
    private String targetMotif; // Default motif

    // Artifact tracking - maps slot number (0-2) to artifact info
    private Map<Integer, ArtifactSlot> artifactSlots = new HashMap<>();

    public KickerSubsystem() {
        RobotMap robotMap = RobotMap.getInstance();

        // Initialize all three slots
        slots[0] = new Slot(
                robotMap.KICKER_0,
                robotMap.COLOR_SENSOR_0_PRIMARY,
                robotMap.COLOR_SENSOR_0_BACKUP,
                0
        );

        slots[1] = new Slot(
                robotMap.KICKER_1,
                robotMap.COLOR_SENSOR_1_PRIMARY,
                robotMap.COLOR_SENSOR_1_BACKUP,
                1
        );

        slots[2] = new Slot(
                robotMap.KICKER_2,
                robotMap.COLOR_SENSOR_2_PRIMARY,
                robotMap.COLOR_SENSOR_2_BACKUP,
                2
        );

        // Initialize artifact tracking
        artifactSlots.put(0, new ArtifactSlot());
        artifactSlots.put(1, new ArtifactSlot());
        artifactSlots.put(2, new ArtifactSlot());
    }

    //----------------------ARTIFACT TRACKING---------------------//

    /**
     * Updates artifact tracking for all slots based on sensor readings
     * ONLY CALL THIS WHEN INTAKING (to minimize I2C calls)
     */
    public void updateArtifactTracking() {
        for (int i = 0; i < 3; i++) {
            ArtifactSlot artifactSlot = artifactSlots.get(i);
            Slot slot = slots[i];

            if (artifactSlot == null || slot == null) continue;

            boolean artifactPresent = slot.hasArtifact();

            if (artifactPresent) {
                // Artifact detected in slot
                if (!artifactSlot.hasConfirmedArtifact()) {
                    // New artifact detected
                    ArtifactColor color = slot.detectColor();
                    int confidence = slot.getDetectionConfidence();

                    artifactSlot.setColor(color);
                    artifactSlot.setHasConfirmedArtifact(true);
                    artifactSlot.setDetectionConfidence(confidence);

                    String confidenceStr = confidence == 2 ? "BOTH sensors" : "ONE sensor";
                    Log.i("Kicker", "Artifact detected in slot " + i + ": " + color +
                            " (" + confidenceStr + ")");
                } else {
                    // Update confidence for existing artifact
                    int confidence = slot.getDetectionConfidence();
                    artifactSlot.setDetectionConfidence(confidence);
                }
            } else {
                // No artifact detected
                if (artifactSlot.hasConfirmedArtifact()) {
                    // Artifact has left the slot (kicked out)
                    Log.i("Kicker", "Artifact left slot " + i);
                    artifactSlot.clear();
                }
            }
        }
    }

    /**
     * Updates a specific slot's artifact tracking
     * Use this for targeted updates instead of scanning all slots (to minimize I2C calls)
     */
    public void updateSlotTracking(int slotNumber) {
        if (slotNumber < 0 || slotNumber >= 3) return;

        ArtifactSlot artifactSlot = artifactSlots.get(slotNumber);
        Slot slot = slots[slotNumber];

        if (artifactSlot == null || slot == null) return;

        boolean artifactPresent = slot.hasArtifact();

        if (artifactPresent) {
            if (!artifactSlot.hasConfirmedArtifact()) {
                ArtifactColor color = slot.detectColor();
                int confidence = slot.getDetectionConfidence();

                artifactSlot.setColor(color);
                artifactSlot.setHasConfirmedArtifact(true);
                artifactSlot.setDetectionConfidence(confidence);

                String confidenceStr = confidence == 2 ? "BOTH sensors" : "ONE sensor";
                Log.i("Kicker", "Artifact detected in slot " + slotNumber + ": " + color +
                        " (" + confidenceStr + ")");
            }
        } else {
            if (artifactSlot.hasConfirmedArtifact()) {
                Log.i("Kicker", "Artifact left slot " + slotNumber);
                artifactSlot.clear();
            }
        }
    }

    /**
     * Clears artifact tracking for a specific slot
     * Call this after kicking to mark the slot as empty
     */
    public void clearSlotTracking(int slotNumber) {
        ArtifactSlot artifactSlot = artifactSlots.get(slotNumber);
        if (artifactSlot != null) {
            artifactSlot.clear();
            Log.i("Kicker", "Cleared tracking for slot " + slotNumber);
        }
    }

    /**
     * Finds the slot number containing the specified color
     */
    public int findArtifactSlot(ArtifactColor targetColor) {
        for (int i = 0; i < 3; i++) {
            ArtifactSlot slot = artifactSlots.get(i);
            if (slot != null && slot.getColor() == targetColor && slot.hasConfirmedArtifact()) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Gets artifact count across all slots
     */
    public int getArtifactCount() {
        int count = 0;
        for (ArtifactSlot slot : artifactSlots.values()) {
            if (slot.hasConfirmedArtifact()) count++;
        }
        return count;
    }

    /**
     * Gets the artifact slot info for a specific slot
     */
    public ArtifactSlot getArtifactSlot(int slotNumber) {
        return artifactSlots.get(slotNumber);
    }

    /**
     * Checks if a slot has a confirmed artifact
     */
    public boolean hasArtifactInSlot(int slotNumber) {
        ArtifactSlot slot = artifactSlots.get(slotNumber);
        return slot != null && slot.hasConfirmedArtifact();
    }

    /**
     * Checks if all slots are full
     */
    public boolean isFull() {
        return getArtifactCount() >= 3;
    }

    /**
     * Checks if all slots are empty
     */
    public boolean isEmpty() {
        return getArtifactCount() == 0;
    }

    /**
     * Resets all artifact tracking
     */
    public void resetArtifactTracking() {
        for (ArtifactSlot slot : artifactSlots.values()) {
            slot.clear();
        }
        Log.i("Kicker", "Reset all artifact tracking");
    }

    //------------------------MOTIF MATCHING---------------------------//

    /**
     * Set the target motif from AprilTag detection
     */
    public void setTargetPattern(String motif) {
        // targetMotif = vision.getTargetMotif()
        this.targetMotif = motif.toUpperCase();
        Log.i("Kicker", "Target pattern set to: " + this.targetMotif);
    }

    /**
     * Get current tracked pattern across all slots
     * Uses artifact tracking, not live sensor reads
     */
    public String getTrackedPattern() {
        StringBuilder pattern = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            ArtifactSlot slot = artifactSlots.get(i);
            if (slot != null && slot.hasConfirmedArtifact()) {
                ArtifactColor color = slot.getColor();
                pattern.append(color == ArtifactColor.PURPLE ? 'P' :
                        color == ArtifactColor.GREEN ? 'G' : 'U');
            } else {
                pattern.append('-'); // Empty slot
            }
        }
        return pattern.toString();
    }

    /**
     * Check if a specific slot matches its target based on tracked artifacts
     */
    public boolean slotMatchesTarget(int slotIndex) {
        if (slotIndex < 0 || slotIndex >= 3 || targetMotif.length() != 3) {
            return false;
        }

        ArtifactSlot slot = artifactSlots.get(slotIndex);
        if (slot == null || !slot.hasConfirmedArtifact()) {
            return false; // No artifact to match
        }

        ArtifactColor detected = slot.getColor();
        char target = targetMotif.charAt(slotIndex);

        if (target == 'P' && detected == ArtifactColor.PURPLE) return true;
        if (target == 'G' && detected == ArtifactColor.GREEN) return true;

        return false;
    }

    //------------------------MODE & CONTROL------------------------//

    /**
     * Set the operating mode
     */
    public void setMode(OperatingMode mode) {
        this.currentMode = mode;
        Log.i("Kicker", "Mode set to: " + mode);
    }

    /**
     * Get the current operating mode
     */
    public OperatingMode getMode() {
        return currentMode;
    }

    /**
     * Kick a specific slot
     */
    public void kickSlot(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < 3) {
            slots[slotIndex].kick();
            Log.i("Kicker", "Kicking slot " + slotIndex);
        }
    }

    /**
     * Reset a specific slot to idle position
     */
    public void resetSlot(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < 3) {
            slots[slotIndex].reset();
        }
    }

    /**
     * Reset all slots to idle position
     */
    public void resetAllSlots() {
        for (Slot slot : slots) {
            slot.reset();
        }
    }

    /**
     * Get color detected in a specific slot (tracked, not live)
     */
    public ArtifactColor getSlotColor(int slotIndex) {
        ArtifactSlot slot = artifactSlots.get(slotIndex);
        if (slot != null && slot.hasConfirmedArtifact()) {
            return slot.getColor();
        }
        return ArtifactColor.UNKNOWN;
    }

    public void reset() {
        resetAllSlots();
        resetArtifactTracking();
    }


    @Override
    public void periodic() {
        // Intentionally empty - we don't want to scan sensors every loop
        // Only scan when explicitly called during intakes
    }


    @Override
    public void onAutonomousInit() {
        resetAllSlots();
        resetArtifactTracking();
        setMode(OperatingMode.AUTO);
        Log.i("Kicker", "Autonomous initialized");
    }

    @Override
    public void onTeleopInit() {
        resetAllSlots();
        resetArtifactTracking();
        setMode(OperatingMode.TELEOP);
        Log.i("Kicker", "Teleop initialized");
    }

    @Override
    public void onTestInit() {
        resetAllSlots();
        resetArtifactTracking();
        setMode(OperatingMode.TELEOP);
        Log.i("Kicker", "Test mode initialized");
    }

    @Override
    public void onAutonomousPeriodic() {
        // Intentionally empty - artifact tracking is done explicitly
        // during intakes to minimize I2C calls
    }

    @Override
    public void onTeleopPeriodic() {
        // Intentionally empty - artifact tracking is done explicitly
        // during intakes to minimize I2C calls
    }

    @Override
    public void onTestPeriodic() {
        // Intentionally empty
    }

    @Override
    public void onDisable() {
        resetAllSlots();
        Log.i("Kicker", "Disabled - all slots reset");
    }
}