package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveConstants.*;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandbase.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.MotorState;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.ATVision;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;


import java.util.ArrayList;
import java.util.List;

public class Enigma extends Robot {
    public Alliance alliance;

    private static Enigma INSTANCE;

    // Hardware and telemetry references
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private LynxModule hub;

    // Subsystems
    private Drive drive;
    private Shooter shooter;
    public Intake intake;

    // List to track all subsystems for periodic updates
    private final List<SubsystemTemplate> subsystems = new ArrayList<>();

    private GamepadEx driveController;
    private GamepadEx manipController;

    private static final double DRIVE_SENSITIVITY = 1.1;
    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRIGGER_DEADZONE = 0.1;
    private static final double ROTATION_DAMPEN = 0.9;


    private Enigma() {
        reset();
        robotInit();
        Log.i("Enigma", "===============ROBOT CREATED SUCCESSFULLY===============");
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

    // Initializes all subsystems and adds them to the list. New subsystems should be added here.
    private void robotInit() {
        subsystems.clear();
        subsystems.add(Drive.getInstance().initialize());
        subsystems.add(ATVision.getInstance().initialize());
        registerSubsystems();
    }

    private void registerSubsystems() {
        for (SubsystemBase s : subsystems) {
            register(s);
        }
    }

    /**
     * Initialize all subsystems
     */
    private void initSubsystems() {
        subsystems.clear();

        drive = new Drive();
        subsystems.add(drive);

        telemetry.addData("Subsystems", "Initialized (" + subsystems.size() + ")");
    }

    public void teleopInit(Telemetry telemetry, HardwareMap hardwareMap, Gamepad drive, Gamepad manip) {
        reset();
        registerSubsystems();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driveController = new GamepadEx(drive);
        manipController = new GamepadEx(manip);

        subsystems.forEach(SubsystemTemplate::onTeleopInit);

        Drive.getInstance().setDefaultCommand(new TeleopDriveCommand(
                () -> applyResponseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY),
                () -> -applyResponseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY),
                () -> -applyResponseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY) * ROTATION_DAMPEN
        ));

            new Trigger(() -> driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                    .whenActive(Drive.getInstance()::enableSlowMode)
                    .whenInactive(Drive.getInstance()::disableSlowMode);

        driveController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(Drive.getInstance()::autoAimHeadingFAR);

        driveController.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(Drive.getInstance()::autoAimHeadingCLOSE);

        driveController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(Drive.getInstance()::lockCurrentHeading);

        manipController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(Intake.ActiveStopIntake());

        manipController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(Shooter.getInstance()::autoAim);

        manipController.getGamepadButton(GamepadKeys.Button.X).whileActiveContinuous(
            new InstantCommand(() -> intake.setIntake(MotorState.REVERSE))
        );

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

    public Drive getDriveSubsystem() {
        return drive;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }


    //-----------------------HELPERS-----------------------//
    // Response Curve Method
    private double applyResponseCurve(double input, double scale) {
        // Limit Input to 1 (MAX) and -1 (MIN)
        input = Math.max(-1, Math.min(1, input));

        // Apply Response Curve
        double output = Math.signum(input) * Math.pow(Math.abs(input), scale);

        return output;
    }

}
