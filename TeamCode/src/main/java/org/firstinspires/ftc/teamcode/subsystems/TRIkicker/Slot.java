package org.firstinspires.ftc.teamcode.subsystems.TRIkicker;

import static org.firstinspires.ftc.teamcode.subsystems.TRIkicker.ColorConstants.GREEN_THRESHOLD;
import static org.firstinspires.ftc.teamcode.subsystems.TRIkicker.ColorConstants.PURPLE_BLUE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.subsystems.TRIkicker.ColorConstants.PURPLE_MIN_RATIO;
import static org.firstinspires.ftc.teamcode.subsystems.TRIkicker.ColorConstants.PURPLE_RED_THRESHOLD;
import static org.firstinspires.ftc.teamcode.subsystems.TRIkicker.TRIConstants.IDLE_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.TRIkicker.TRIConstants.KICKED_POSITION;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.enums.ArtifactColor;

// Slot class to hold sensors and servo for each slot
public class Slot {
    Servo kicker;
    RevColorSensorV3 primarySensor;
    RevColorSensorV3 backupSensor;
    int slotNumber;

    Slot(Servo kicker, RevColorSensorV3 primary, RevColorSensorV3 backup, int number) {
        this.kicker = kicker;
        this.primarySensor = primary;
        this.backupSensor = backup;
        this.slotNumber = number;
        kicker.setPosition(IDLE_POSITION);
    }

    /**
     * Detects color with fallback to backup sensor
     * Only call when needed (to minimize I2C calls)
     */
    ArtifactColor detectColor() {
        // Try primary sensor first
        ArtifactColor primaryColor = getColorFromSensor(primarySensor);

        // If primary fails, use backup
        if (primaryColor == ArtifactColor.UNKNOWN) {
            return getColorFromSensor(backupSensor);
        }

        return primaryColor;
    }

    /**
     * Gets detection confidence (0, 1, or 2 sensors detecting)
     * Only call when needed (to minimize I2C calls)
     */
    int getDetectionConfidence() {
        int confidence = 0;
        if (isArtifactPresent(primarySensor)) confidence++;
        if (isArtifactPresent(backupSensor)) confidence++;
        return confidence;
    }

    /**
     * Checks if artifact is present in slot
     * Returns true if EITHER sensor detects a ball
     */
    boolean hasArtifact() {
        return isArtifactPresent(primarySensor) || isArtifactPresent(backupSensor);
    }

    /**
     * Generic artifact presence check for a sensor
     */
    private boolean isArtifactPresent(RevColorSensorV3 sensor) {
        int green = sensor.green();
        int red = sensor.red();
        int blue = sensor.blue();

        // Artifact present if any color value exceeds threshold
        return (green > GREEN_THRESHOLD) ||
                (red > PURPLE_RED_THRESHOLD && blue > PURPLE_BLUE_THRESHOLD);
    }

    /**
     * Classifies color from a single sensor
     */
    private ArtifactColor getColorFromSensor(RevColorSensorV3 sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Detect GREEN - green channel dominant
        if (green > GREEN_THRESHOLD && green > red && green > blue) {
            return ArtifactColor.GREEN;
        }

        // Detect PURPLE - combination of red and blue, low green
        if (red > PURPLE_RED_THRESHOLD && blue > PURPLE_BLUE_THRESHOLD) {
            // Check that red and blue are similar (purple is balanced)
            double ratio = Math.min(red, blue) / (double) Math.max(red, blue);

            // Purple should have low green relative to red/blue
            int avgRedBlue = (red + blue) / 2;

            if (ratio > PURPLE_MIN_RATIO && green < avgRedBlue) {
                return ArtifactColor.PURPLE;
            }
        }

        return ArtifactColor.UNKNOWN;
    }

    void kick() {
        kicker.setPosition(KICKED_POSITION);
    }

    void reset() {
        kicker.setPosition(IDLE_POSITION);
    }
}