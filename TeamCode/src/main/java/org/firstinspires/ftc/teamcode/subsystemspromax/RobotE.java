package org.firstinspires.ftc.teamcode.subsystemspromax;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc.InertialLaunchCore;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class RobotE {
    // Subsystems
    public final Intake intake;
    public final InertialLaunchCore ilc;
    public final Follower follower;

    // Hardware
    private final LynxModule hub;

    // Telemetry
    private Telemetry telemetry;

    // Alliance
    public Alliance alliance;

    // State
    public static Pose endPose;
    private final Timer loopTimer = new Timer();

    // Heading lock
    private double targetHeading = Math.toRadians(180);
    private PIDFController headingLockPID;
    private boolean headingLockEnabled = false;

    // Gamepads (only for teleop)
    private GamepadEx driveController;
    private GamepadEx manipController;

    // Control sensitivity
    private static final double DRIVE_SENSITIVITY = 1.1;
    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRIGGER_DEADZONE = 0.1;
    private static final double ROTATION_DAMPEN = 0.9;
    private static final double JOYSTICK_DEAD_ZONE = 0.1;

    private boolean slowModeEnabled = false;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;

    public RobotE(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        // Initialize subsystems
        intake = new Intake(hardwareMap);
        ilc = new InertialLaunchCore(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // Initialize hardware
        hub = hardwareMap.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Initialize heading lock PID
        headingLockPID = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        loopTimer.resetTimer();
    }

    // Autonomous init
    public void onAutoInit(Telemetry telemetry, HardwareMap hardwareMap, Pose startPose) {
        // Setup telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Set starting pose
        follower.setStartingPose(startPose);

        // Disable heading lock for auto (path following handles heading)
        headingLockEnabled = false;

        // Reset subsystems
        intake.setIDLE();
        ilc.forceIdle();

        this.telemetry.addLine("Auto Initialized");
        this.telemetry.addData("Alliance", alliance);
        this.telemetry.addData("Start Pose", startPose);
        this.telemetry.update();
    }

    public void onTeleopInit(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        // Setup telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        this.driveController = new GamepadEx(gamepad1);
        this.manipController = new GamepadEx(gamepad2);

        // Set starting pose (use end pose from auto if available)
        if (endPose != null) {
            follower.setStartingPose(endPose);
        }

        // Enable heading lock for teleop
        headingLockEnabled = false; // Start disabled, user can toggle
        targetHeading = follower.getPose().getHeading();


        // Reset subsystems
        intake.setIDLE();
        ilc.forceIdle();

        bindControls();

        this.telemetry.addLine("Teleop Initialized");
        this.telemetry.addData("Alliance", alliance);
        this.telemetry.update();
    }

    private void bindControls() {
        // Slow mode toggle
        new Trigger(() ->
                driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(() -> slowModeEnabled = true)
                .whenInactive(() -> slowModeEnabled = false);

        // Reset heading to 180° (or 0° depending on alliance)
        driveController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> {
                    double resetAngle = alliance == Alliance.BLUE ? Math.toRadians(180) : 0;
                    follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), resetAngle));
                    targetHeading = resetAngle;
                });

        // Toggle heading lock
        driveController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    headingLockEnabled = !headingLockEnabled;
                    if (headingLockEnabled) {
                        targetHeading = follower.getPose().getHeading();
                    }
                });

        // Toggle intake
        manipController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> {
                    if (intake.getIntakeState() == IntakeState.IDLE) {
                        intake.setIntake(IntakeState.INTAKING);
                    } else {
                        intake.setIDLE();
                    }
                });

        // Stop intake
        manipController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> intake.setIDLE());

        // Reverse intake - hold to reverse
        manipController.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(() -> intake.setIntake(IntakeState.REVERSE))
                .whenReleased(() -> intake.setIntake(IntakeState.INTAKING));

        // Spin up ILC and shoot sequence
        manipController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    if (ilc.isIdle()) {
                        // Start spinup
                        ilc.startSpinup();
                    } else if (ilc.isReady()) {
                        // If ready, shoot
                        ilc.shoot();
                        intake.setIntake(IntakeState.TRANSFERRING);
                    }
                });

        // Stop ILC
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    if (!ilc.isIdle()) {
                        ilc.forceIdle();
                        intake.setIDLE();
                    }
                });

        // Manual gate control
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> ilc.gateLaunch());

        manipController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> ilc.gateStop());
    }

    public void handleTeleopControls() {
        if (driveController == null) return;

        // Read all buttons
        driveController.readButtons();
        manipController.readButtons();

        double drive = -applyResponseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY);
        double strafe = -applyResponseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY);
        double turn = -applyResponseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY) * ROTATION_DAMPEN;

        // Apply slow mode if enabled
        if (slowModeEnabled) {
            drive *= SLOW_MODE_MULTIPLIER;
            strafe *= SLOW_MODE_MULTIPLIER;
            turn *= SLOW_MODE_MULTIPLIER;
        }

        // Heading lock logic
        if (headingLockEnabled) {
            double error = targetHeading - follower.getPose().getHeading();
            headingLockPID.setCoefficients(follower.constants.coefficientsHeadingPIDF);
            headingLockPID.updateError(error);
            double headingCorrection = headingLockPID.run();

            follower.setTeleOpDrive(drive, strafe, headingCorrection);
        } else {
            follower.setTeleOpDrive(drive, strafe, turn);
        }
    }

    public void periodic() {
        follower.update();
        ilc.periodic();
        intake.periodic();

        if (loopTimer.getElapsedTime() % 5 == 0) {
            hub.clearBulkCache();
        }

        double error = targetHeading - follower.getHeading();
        headingLockPID.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        headingLockPID.updateError(error);

    }

    public void stop() {
        endPose = follower.getPose();
    }


    // Response Curve Method
    private double applyResponseCurve(double input, double scale) {
        // Limit Input to 1 (MAX) and -1 (MIN)
        input = Math.max(-1, Math.min(1, input));

        // Apply Response Curve
        return Math.signum(input) * Math.pow(Math.abs(input), scale);
    }
}
