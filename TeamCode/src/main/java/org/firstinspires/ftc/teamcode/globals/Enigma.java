package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;


import java.util.ArrayList;
import java.util.List;

public class Enigma {
    public Alliance alliance;

    private static Enigma INSTANCE;

    // Hardware and telemetry references
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private LynxModule hub;

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
    public void init(HardwareMap hardwareMap, Alliance alliance) {
        this.hardwareMap = hardwareMap;


        // Initialize subsystems
        initSubsystems();

        hub = hardwareMap.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

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

    //------------------------SETTERS------------------------//

    public void setShootHeading() {
        if (alliance == Alliance.BLUE) {
            shootHeadingCLOSE = Math.toRadians(135); // TODO: tune
            shootHeadingFAR = Math.toRadians(120); //  TODO: tune
        }
        else if (alliance == Alliance.RED)
            shootHeadingCLOSE = Math.toRadians(135); // TODO: tune
        shootHeadingFAR = Math.toRadians(120); //  TODO: tune
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
