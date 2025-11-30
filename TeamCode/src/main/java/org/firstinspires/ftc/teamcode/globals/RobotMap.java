package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;


public class RobotMap {
    private HardwareMap hardwareMap;

    private static RobotMap instance = null;

    // Returns an instance of this
    public static RobotMap getInstance() {
        if (instance == null) {
            instance = new RobotMap();
        }
        return instance;
    }


    // Drive hardware
    public DcMotorEx FRONT_RIGHT;
    public DcMotorEx BACK_RIGHT;
    public DcMotorEx FRONT_LEFT;
    public DcMotorEx BACK_LEFT;

    // Arducam
    public WebcamName ARDUCAM;

    // Intake hardware
    public DcMotorEx INTAKE;
    public RevColorSensorV3 PROXIMITY_SENSOR;

//    // Kicker subsystem hardware
//    public Servo KICKER_0;
//    public Servo KICKER_1;
//    public Servo KICKER_2;
//
//    public RevColorSensorV3 COLOR_SENSOR_0_PRIMARY;
//    public RevColorSensorV3 COLOR_SENSOR_0_BACKUP;
//    public RevColorSensorV3 COLOR_SENSOR_1_PRIMARY;
//    public RevColorSensorV3 COLOR_SENSOR_1_BACKUP;
//    public RevColorSensorV3 COLOR_SENSOR_2_PRIMARY;
//    public RevColorSensorV3 COLOR_SENSOR_2_BACKUP;

    // Shooter hardware
    public DcMotorEx SHOOTER_0;
    public DcMotorEx SHOOTER_1;

    public Servo HOOD;

    private RobotMap() {
    }

    public void init(final HardwareMap hardwareMap) {
        // Initialize drive motors
        FRONT_RIGHT = hardwareMap.get(DcMotorEx.class, "rf");
        BACK_RIGHT = hardwareMap.get(DcMotorEx.class, "rr");
        FRONT_LEFT = hardwareMap.get(DcMotorEx.class, "lf");
        BACK_LEFT = hardwareMap.get(DcMotorEx.class, "lr");

        // Initialize arducam
        ARDUCAM = hardwareMap.get(WebcamName.class, "arducam");

        // Initialize intake motors
        INTAKE = hardwareMap.get(DcMotorEx.class, "intake");
        PROXIMITY_SENSOR = hardwareMap.get(RevColorSensorV3.class, "proximitySensor");


//        // Initialize kicker servos
//        KICKER_0 = hardwareMap.get(Servo.class, "kicker0");
//        KICKER_1 = hardwareMap.get(Servo.class, "kicker1");
//        KICKER_2 = hardwareMap.get(Servo.class, "kicker2");
//
//        // Initialize color sensors
//        COLOR_SENSOR_0_PRIMARY = hardwareMap.get(RevColorSensorV3.class, "colorSensor0Primary");
//        COLOR_SENSOR_0_BACKUP = hardwareMap.get(RevColorSensorV3.class, "colorSensor0Backup");
//        COLOR_SENSOR_1_PRIMARY = hardwareMap.get(RevColorSensorV3.class, "colorSensor1Primary");
//        COLOR_SENSOR_1_BACKUP = hardwareMap.get(RevColorSensorV3.class, "colorSensor1Backup");
//        COLOR_SENSOR_2_PRIMARY = hardwareMap.get(RevColorSensorV3.class, "colorSensor2Primary");
//        COLOR_SENSOR_2_BACKUP = hardwareMap.get(RevColorSensorV3.class, "colorSensor2Backup");

        // Initialize shooter motors
        SHOOTER_0 = hardwareMap.get(DcMotorEx.class, "shooter0");
        SHOOTER_1 = hardwareMap.get(DcMotorEx.class, "shooter1");

        // Initialize hood servo
        HOOD = hardwareMap.get(Servo.class, "hood");
    }


    // Get hardwareMap instance
    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
    }

    // Get hubs
    public List<LynxModule> getLynxModules() {
        return hardwareMap.getAll(LynxModule.class);
    }
}
