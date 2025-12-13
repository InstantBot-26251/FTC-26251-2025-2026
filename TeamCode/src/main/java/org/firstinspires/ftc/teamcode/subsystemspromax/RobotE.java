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
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands.ShootSequenceCommand;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc.InertialLaunchCore;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class RobotE {
    // Subsystems
    public final Intake intake;
    public final InertialLaunchCore ilc;
    public final Follower follower;

    // Commands
    private IntakeCommand intakeCommand;
    private ShootSequenceCommand shootCommand;

    // Hardware
    private final LynxModule hub;

    // Telemetry
    public Telemetry telemetry;

    // Alliance
    public Alliance alliance;

    // State
    public static Pose endPose;
    private final Timer loopTimer = new Timer();

    // Heading lock
    private double targetHeading = Math.toRadians(180);
    private PIDFController headingLockPID;
    public boolean headingLockEnabled = false;

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

    private final PIDFController headingController;

    private final double goalX;
    private final double goalY;

    private double targetHeadingL = 0.0;
    private double lockedHeading = 0.0;
    private boolean isLocked = false;

    private static final double HEADING_TOLERANCE = Math.toRadians(2.0);

    public RobotE(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        // Initialize subsystems
        intake = new Intake(hardwareMap);
        ilc = new InertialLaunchCore(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // Initialize commands
        intakeCommand = new IntakeCommand(intake);
        shootCommand = new ShootSequenceCommand(ilc, intake);

        // Initialize hardware
        hub = hardwareMap.getAll(LynxModule.class).get(0);
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        goalX = 144;
        goalY = 144;

        // Initialize heading lock PID
        headingLockPID = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        // Initialize aim PID
        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

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


        follower.setPose(new Pose(0, 0, Math.toRadians(180)));
        follower.startTeleopDrive();

        // Enable heading lock for teleop
        headingLockEnabled = false; // Start disabled, user can toggle
        targetHeading = follower.getPose().getHeading();

        // Reset subsystems
        intake.setIDLE();
        ilc.forceIdle();
//
//        bindControls();

        this.telemetry.addLine("Teleop Initialized");
        this.telemetry.addData("Alliance", alliance);
        this.telemetry.update();
    }

    public void bindControls() {
        // Slow mode toggle
        new Trigger(() ->
                driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(() -> slowModeEnabled = true)
                .whenInactive(() -> slowModeEnabled = false);

        // Reset heading to 180° (or 0° depending on alliance)
        driveController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> resetHeading()));

        // Toggle heading lock
        driveController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> toggleHeadingLock()));

        // Auto Aim
        driveController.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new InstantCommand(() -> autoAim()))
                .whenReleased(new InstantCommand(() -> disableAutoAim()));

        // === INTAKE CONTROLS ===
        // Toggle intake command - B button
        manipController.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(intakeCommand);

        // Stop intake - X button
        manipController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    intakeCommand.cancel();
                    intake.setIDLE();
                }));

        // Manual reverse - hold Y to reverse
        manipController.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new InstantCommand(() -> intake.setIntake(IntakeState.REVERSE)))
                .whenReleased(new InstantCommand(() -> {
                    // If intake command was running, restart it, otherwise go idle
                    if (intakeCommand.isScheduled()) {
                        intake.setIntake(IntakeState.INTAKING);
                    } else {
                        intake.setIDLE();
                    }
                }));

        // === SHOOTING CONTROLS ===
        // Full shooting sequence - A button
        manipController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(shootCommand);

        // Emergency stop / force idle - Left bumper
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    shootCommand.cancel();
                    ilc.forceIdle();
                    intake.setIDLE();
                }));

        // Manual ILC control (for testing/tuning)
        // DPAD_UP: Start spinup only
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(() -> ilc.startSpinup());

        // DPAD_DOWN: Manual shoot (if ready)
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> ilc.shoot());

        // DPAD_RIGHT: Stop shooting
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> ilc.stopShooting());
    }

    public void handleTeleopControls() {
        // Read all buttons
        driveController.readButtons();
        manipController.readButtons();

        double drive = applyResponseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY);
        double strafe = applyResponseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY);
        double turn = applyResponseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY) * ROTATION_DAMPEN;

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

        CommandScheduler.getInstance().run();
    }

    public void stop() {
        endPose = follower.getPose();
        intakeCommand.cancel();
        shootCommand.cancel();
    }

    private void resetHeading() {
        double resetAngle = alliance == Alliance.BLUE ? Math.toRadians(180) : 0;
        follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), resetAngle));
        targetHeading = resetAngle;
    }

    private void toggleHeadingLock() {
        headingLockEnabled = !headingLockEnabled;
        if (headingLockEnabled) {
            targetHeading = follower.getPose().getHeading();
        }
    }

    // Response Curve Method
    private double applyResponseCurve(double input, double scale) {
        // Limit Input to 1 (MAX) and -1 (MIN)
        input = Math.max(-1, Math.min(1, input));

        // Apply Response Curve
        return Math.signum(input) * Math.pow(Math.abs(input), scale);
    }

    public double autoAim() {
        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        double error;

        // If not locked yet, calculate heading to goal
        if (!isLocked) {
            targetHeading = Math.atan2(goalY - robotY, goalX - robotX);
            error = angleWrap(targetHeading - robotHeading);

            // Check if aligned - lock the heading
            if (Math.abs(error) < HEADING_TOLERANCE) {
                lockedHeading = robotHeading;
                isLocked = true;
            }
        } else {
            // Maintain locked heading
            error = angleWrap(lockedHeading - robotHeading);
        }

        // Calculate turn power
        headingController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        headingController.updateError(error);

        return clampTurn(headingController.run());
    }

    public void disableAutoAim() {
        isLocked = false;
        headingController.reset();
    }

    public boolean isLocked() {
        return isLocked;
    }

    public double getTargetDistance() {
//        // Try vision first
//        double visionDistance = ilc.getDistance();
//        if (visionDistance > 0) {
//            return visionDistance;
//        }

        // Fallback to odometry
        Pose pose = follower.getPose();
        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double clampTurn(double turn) {
        return Math.max(-0.5, Math.min(0.5, turn));
    }
}