package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;

public class Enigma {

    private static Enigma INSTANCE;

    // Hardware and telemetry references
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Subsystems
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
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

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

        driveSubsystem = new DriveSubsystem();
        subsystems.add(driveSubsystem);

        telemetry.addData("Subsystems", "Initialized (" + subsystems.size() + ")");
    }


    public void periodic() {
        for (SubsystemBase subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    //------------------------GETTERS------------------------//

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }


}
