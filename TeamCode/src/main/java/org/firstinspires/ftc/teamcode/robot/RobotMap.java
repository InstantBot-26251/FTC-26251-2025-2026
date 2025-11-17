package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotMap {

    private static RobotMap INSTANCE;

    // Kicker hardware
    public Servo KICKER_0;
    public Servo KICKER_1;
    public Servo KICKER_2;

    public RevColorSensorV3 COLOR_SENSOR_0_PRIMARY;
    public RevColorSensorV3 COLOR_SENSOR_0_BACKUP;
    public RevColorSensorV3 COLOR_SENSOR_1_PRIMARY;
    public RevColorSensorV3 COLOR_SENSOR_1_BACKUP;
    public RevColorSensorV3 COLOR_SENSOR_2_PRIMARY;
    public RevColorSensorV3 COLOR_SENSOR_2_BACKUP;

    private RobotMap() {
    }

    public static RobotMap getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotMap();
        }
        return INSTANCE;
    }

    /**
     * Initialize all hardware from hardware map
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize kicker servos
        KICKER_0 = hardwareMap.get(Servo.class, "kicker0");
        KICKER_1 = hardwareMap.get(Servo.class, "kicker1");
        KICKER_2 = hardwareMap.get(Servo.class, "kicker2");

        // Initialize color sensors
        COLOR_SENSOR_0_PRIMARY = hardwareMap.get(RevColorSensorV3.class, "colorSensor0Primary");
        COLOR_SENSOR_0_BACKUP = hardwareMap.get(RevColorSensorV3.class, "colorSensor0Backup");
        COLOR_SENSOR_1_PRIMARY = hardwareMap.get(RevColorSensorV3.class, "colorSensor1Primary");
        COLOR_SENSOR_1_BACKUP = hardwareMap.get(RevColorSensorV3.class, "colorSensor1Backup");
        COLOR_SENSOR_2_PRIMARY = hardwareMap.get(RevColorSensorV3.class, "colorSensor2Primary");
        COLOR_SENSOR_2_BACKUP = hardwareMap.get(RevColorSensorV3.class, "colorSensor2Backup");

    }
}
