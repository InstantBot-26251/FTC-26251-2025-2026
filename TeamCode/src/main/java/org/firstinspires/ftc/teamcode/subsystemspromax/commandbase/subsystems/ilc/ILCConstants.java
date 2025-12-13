package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ILCConstants {
    public static double kS = 0.08, kV = 0.00039, kP = 0.001;

    // Linear Interpolation Lookup Table (LUT)
    // TODO: Test and Tune
    public static double[][] distanceRPMLUT = {
            {12.0, 1500.0},  // Close range
            {24.0, 1800.0},
            {36.0, 2100.0},
            {48.0, 2400.0}, // Mid range
            {60.0, 2700.0},
            {72.0, 3000.0},
            {84.0, 3300.0},
            {96.0, 3600.0} // Long range
    };

    // GATE CONSTANTS
//    public static double STOP_POSITION = 1;
//    public static double SHOOT_POSITION = 0;
//    public static double GATE_OPEN_TIME = 1;
//    public static double GATE_CLOSE_TIME = 0.3;

    // ILC CONSTANTS
    public static double flywheelVelocity = 0;
    public static double MIN_FLYWHEEL_RPM = 800;
    public static double FLYWHEEL_MAX_SPINUP_TIME = 2;
    public static double TRANSFER_REVERSE_TIME = 0.1; // TUNE
    public static double TRANSFER_REVERSE_POWER = -0.5; // TUNE
    public static double TRANSFER_SHOOT_POWER = 1.0; // Full speed forward
}
