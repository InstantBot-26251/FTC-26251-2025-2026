package org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // Shooter power constants
    public static double SHOOTER_IDLE_POWER = 0.0;
    public static double SHOOTER_MAX_POWER = 1.0;

    // Hood angle constants
    public static double HOOD_MIN_POSITION = 0.0;  // Flat/low angle
    public static double HOOD_MAX_POSITION = 1.0;  // Max angle

    // Empirically determined hood positions at various distances
    // TODO: Fill these in after testing! Format: {distance_in_inches, hood_servo_position}
    public static final double[][] HOOD_LOOKUP_TABLE = {
            {30.0, 0.10},   // ~30 inches - Example data, replace with real values once tested, run hoodtesting
            {36.0, 0.15},   // ~3 feet
            {42.0, 0.20},   // ~3.5 feet
            {48.0, 0.25},   // 4 feet
            {54.0, 0.30},   // ~4.5 feet
            {60.0, 0.35},   // 5 feet
            {66.0, 0.40},   // ~5.5 feet
            {72.0, 0.45},   // 6 feet
            {78.0, 0.50},   // ~6.5 feet
            {84.0, 0.55},   // 7 feet
            {90.0, 0.60},   // ~7.5 feet
            {96.0, 0.65},   // 8 feet
            {102.0, 0.70},  // ~8.5 feet
            {108.0, 0.75},  // 9 feet
            {114.0, 0.80},  // ~9.5 feet
            {116.0, 0.85}   // ~116 inches (max range)
    };

}
