package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
    // Heading PID constants
    // TODO: TUNE
    public static final double HEADING_kP = 1.5;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.1;
    public static final double HEADING_kF = 0.0;
    public static double HEADING_TOLERANCE = Math.toRadians(2); // 2 degrees

    // For debugging
    public static double headingError = 0.0;
    public static double headingPower = 0.0;

}
