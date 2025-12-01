package org.firstinspires.ftc.teamcode.commandbase.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    public static double GOAL_POSE_X = 5; // TODO: REPLACE WITH REAL VALUE
    public static double GOAL_POSE_Y = 0;

    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;


    // For auto-aim
    public static double currentHeading = 0.0;
    public static double shootHeadingCLOSE = 0.0; // TODO: Tune: just do localization test and then get the current heading and put it here
    public static double shootHeadingFAR = 0.0; // TODO: Tune: just do localization test and then get the current heading and put it here

}
