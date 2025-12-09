package org.firstinspires.ftc.teamcode.solverslib.globals;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.solverslib.commandbase.commands.AIM;
import org.firstinspires.ftc.teamcode.solverslib.commandbase.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.ilc.InertialLaunchCoreILC;
import org.firstinspires.ftc.teamcode.util.Alliance;


public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    public Alliance alliance;

    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    // Hardware and telemetry references
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Follower follower;

    // Subsystems
    public Drive drive;
    public InertialLaunchCoreILC ilc;
    public Intake intake;


    public GamepadEx driveController;
    private GamepadEx manipController;

    public static final double DRIVE_SENSITIVITY = 1.1;
    public static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRIGGER_DEADZONE = 0.1;
    public static final double ROTATION_DAMPEN = 0.9;


    public void autonomousInit(Telemetry telemetry, HardwareMap hardwareMap) {
        reset();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.hardwareMap = hardwareMap;

        RobotMap.getInstance().init(hardwareMap);

        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize subsystems
        drive = new Drive(hardwareMap);
        ilc = new InertialLaunchCoreILC(hardwareMap);
        intake = new Intake(hardwareMap);

        // Initialize hardware for each subsystem
//        drive.initHardware(hardwareMap);
//        ilc.initHardware(hardwareMap);
//        intake.initHardware(hardwareMap);

        // Register subsystems
        register(drive, ilc, intake);

        Log.i("Robot", "===============Autonomous Initialized==============");
    }

    public void teleopInit(Telemetry telemetry, HardwareMap hardwareMap) {
        reset();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.hardwareMap = hardwareMap;

        RobotMap.getInstance().init(hardwareMap);

        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize subsystems
        drive = new Drive(hardwareMap);
        ilc = new InertialLaunchCoreILC(hardwareMap);
        intake = new Intake(hardwareMap);

        // Initialize hardware
//        drive.initHardware(hardwareMap);
//        ilc.initHardware(hardwareMap);
//        intake.initHardware(hardwareMap);

        // REGISTRATO subsystems
        register(drive, ilc, intake);

        Log.i("Robot", "===============Teleop Initialized==============");
    }

    public void teleopInitWithControls(Telemetry telemetry, HardwareMap hardwareMap, Gamepad driver, Gamepad manip) {
        reset();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.hardwareMap = hardwareMap;

        RobotMap.getInstance().init(hardwareMap);

        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driveController = new GamepadEx(driver);
        manipController = new GamepadEx(manip);

//        // Initialize subsystems
//        drive = Drive.getInstance();
//        shooter = Shooter.getInstance();
//        intake = Intake.getInstance();

        // Initializa subsystems
        drive = new Drive(hardwareMap);
        ilc = new InertialLaunchCoreILC(hardwareMap);
        intake = new Intake(hardwareMap);

        // Initialize hardware for each subsystem
//        drive.initHardware(hardwareMap);
//        shooter.initHardware(hardwareMap, telemetry);
//        intake.initHardware(hardwareMap);

        // Register subsystems
        register(drive, ilc, intake);

        /**
         *  Driver Controls
         */
        drive.setDefaultCommand(new TeleopDriveCommand(
                () -> applyResponseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY),
                () -> -applyResponseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY),
                () -> -applyResponseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY) * ROTATION_DAMPEN
        ));

        new Trigger(() -> driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(drive::enableSlowMode)
                .whenInactive(drive::disableSlowMode);

        // Reset heading
        driveController.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(drive::resetHeading)
        );

        // Lock current heading
        driveController.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> drive.lockCurrentHeading(driveController.getLeftY(), driveController.getLeftX(), driveController.getRightX()))
        );

        // Toggle heading lock
        driveController.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(drive::toggleHeadingLock)
        );

        /**
         * Manipulator Controls
         */

        // Overrides all intake commands and stops intake
        manipController.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                Intake.ActiveStopIntake()
        );

        // Intake reverse
        manipController.getGamepadButton(GamepadKeys.Button.X).whileActiveContinuous(
                new InstantCommand(() -> intake.setIntake(IntakeState.REVERSE))
        );

        // Intake forward
        manipController.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                new InstantCommand(() -> intake.setIntake(IntakeState.FORWARD))
        );

        // Intake transfer - shoot
        manipController.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> intake.setIntake(IntakeState.TRANSFER))
        );

        // Hood and shooter settings auto set
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(ilc::autoAim)
        );

        // FULL AIM LEBRON
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new AIM()
        );

        Log.i("Robot", "===============Teleop Initialized WITH CONTROLS==============");
    }

    //------------------------SETTERS------------------------//

    public void setShootHeading() {
        if (alliance == Alliance.BLUE) {
            shootHeadingCLOSE = Math.toRadians(135);
            shootHeadingFAR = Math.toRadians(120);
        }
        else if (alliance == Alliance.RED) {
            shootHeadingCLOSE = Math.toRadians(135);
            shootHeadingFAR = Math.toRadians(120);
        }
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
    public double applyResponseCurve(double input, double scale) {
        // Limit Input to 1 (MAX) and -1 (MIN)
        input = Math.max(-1, Math.min(1, input));

        // Apply Response Curve
        return Math.signum(input) * Math.pow(Math.abs(input), scale);
    }

}