package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    // INTAKE SPEEDS
    public static double IDLE = 0.0;
    public static double INTAKE = 1;
    public static double TRANSFER = 0.7;
    public static double REVERSE = -1;

    public static double INTAKE_UNJAM_TIME = 1.0; // TODO: Tune
    public static int FULL_THRESHOLD = 3;

    // SENSOR CONSTants
    public static double PROXIMITY_THRESHOLD_CM = 3.0; // TODO: Tune

}
