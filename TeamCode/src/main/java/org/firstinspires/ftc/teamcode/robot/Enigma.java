package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;

public class Enigma {

    private static Enigma INSTANCE;

    // Hardware and telemetry references
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private OpMode opMode;

    // Subsystems
    private KickerSubsystem kickerSubsystem;
    private DriveSubsystem driveSubsystem;

    // List to track all subsystems for periodic updates
    private List<SubsystemBase> subsystems = new ArrayList<>();


    private Enigma() {
        // Intentionally empty - initialization happens in init()
    }

    /**
     * Get the singleton instance
     */
    public static Enigma getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Enigma();
        }
        return INSTANCE;
    }

    /**
     * Initialize the robot with hardware map and telemetry
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, OpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        // Initialize subsystems
        initSubsystems();

        telemetry.addData("Robot", "Initialized");
        telemetry.update();
    }

    /**
     * Initialize all subsystems
     */
    private void initSubsystems() {
        subsystems.clear();

        // Initialize kicker subsystem
        kickerSubsystem = new KickerSubsystem();
        subsystems.add(kickerSubsystem);

        driveSubsystem = new DriveSubsystem();
        subsystems.add(driveSubsystem);

        telemetry.addData("Subsystems", "Initialized (" + subsystems.size() + ")");
    }


    public void periodic() {
        for (SubsystemBase subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    /**
     * Reset the robot - useful for switching between auto and teleop
     */
    public void reset() {
        // Reset kicker subsystem
        if (kickerSubsystem != null) {
            kickerSubsystem.reset();
        }

        telemetry.addData("Robot", "Reset");
    }

    // ===================== GETTERS =====================

    public KickerSubsystem getKickerSubsystem() {
        return kickerSubsystem;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public OpMode getOpMode() {
        return opMode;
    }

}
