package org.firstinspires.ftc.teamcode.commandbase.subsystems.drive;

import android.util.Log;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveConstants.*;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.globals.Enigma;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;

public class Drive extends SubsystemTemplate {

    private static final Drive INSTANCE = new Drive();

    private Telemetry telemetry;

    public static Drive getInstance() {
        return INSTANCE;
    }

    private Follower follower;

    @Override
    public void onAutonomousInit() {
        currentMode = DriveMode.AUTONOMOUS;
        automatedDrive = false;
        slowModeEnabled = false;
        disableHeadingLock();

        // Note: Starting pose should be set by the autonomous OpMode
        // before paths are followed

        Log.i("Drive", "Autonomous initialized - ready for path following");
    }

    @Override
    public void onTeleopInit() {
        telemetry = Enigma.getInstance().getTelemetry();

        Constants.createFollower(RobotMap.getInstance().getHardwareMap());

        // Start teleop drive mode with brake mode enabled
        startTeleopDrive(true);

        // Set default teleop mode
        currentMode = DriveMode.TELEOP_FIELD_CENTRIC;

        // Reset slow mode to default
        slowModeEnabled = false;
        slowModeMultiplier = 0.5;

        follower.setPose(follower.getPose());


        Log.i("Drive", "Teleop initialized - manual control enabled");
    }



    // Drive mode tracking
    public enum DriveMode {
        TELEOP_FIELD_CENTRIC,
        TELEOP_ROBOT_CENTRIC,
        AUTONOMOUS
    }

    private DriveMode currentMode = DriveMode.TELEOP_ROBOT_CENTRIC;

    // Slow mode
    private boolean slowModeEnabled = false;
    private double slowModeMultiplier = 0.5;

    // Track if we're following an automated path in teleop
    private boolean automatedDrive = false;

    private boolean headingLockEnabled = false;
    private double targetHeading = 0.0; // Target heading in radians
    private PIDFController headingPID;


    public Drive() {
        // This is the official Pedro 2.0.4 initialization method
        follower = Constants.createFollower(Enigma.getInstance().getHardwareMap());

        // Set default starting pose
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize PIDF Controller
        headingPID = new PIDFController(new PIDFCoefficients(HEADING_kP, HEADING_kI, HEADING_kD, HEADING_kF));

        Log.i("Drive", "DriveSubsystem initialized with Pedro Pathing 2.0.4");
    }

    public void startTeleopDrive(boolean useBrakeMode) {
        follower.startTeleopDrive(useBrakeMode);
        automatedDrive = false;
        Log.i("Drive", "Started teleop drive mode (brake: " + useBrakeMode + ")");
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
        automatedDrive = false;
        Log.i("Drive", "Started teleop drive mode (default brake mode)");
    }

    //------------------------TELEOP DRIVE------------------------//

    public void setTeleOpDrive(double forwardSpeed, double strafeSpeed, double turnSpeed, boolean robotCentric) {
        if (!automatedDrive) {
            // Apply slow mode multiplier if enabled
            if (slowModeEnabled) {
                forwardSpeed *= slowModeMultiplier;
                strafeSpeed *= slowModeMultiplier;
                turnSpeed *= slowModeMultiplier;
            }

            follower.setTeleOpDrive(forwardSpeed, strafeSpeed, turnSpeed, robotCentric);

            // Update current mode
            currentMode = robotCentric ? DriveMode.TELEOP_ROBOT_CENTRIC : DriveMode.TELEOP_FIELD_CENTRIC;
        }
    }

    /**
     * Field-centric teleop drive
     */
    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        setTeleOpDrive(forwardSpeed, strafeSpeed, turnSpeed, false);
    }

    /**
     * Robot-centric teleop drive
     */
    public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        setTeleOpDrive(forwardSpeed, strafeSpeed, turnSpeed, true);
    }

    //------------------------HEADING LOCK------------------------//

    /**
     * Lock the current heading - robot will maintain this heading until disengaged
     */
    public void lockCurrentHeading() {
        targetHeading = getPose().getHeading();
        headingLockEnabled = true;
//        headingPID.reset();
//        headingPID.setTargetPosition(targetHeading);
        follower.holdPoint(getPose());
        Log.i("Drive", "Heading locked at: " + Math.toDegrees(targetHeading) + " degrees");
    }

    /**
     * Lock to a specific heading in radians
     */
    public void lockHeading(double headingRadians) {
        targetHeading = normalizeAngle(headingRadians);
        headingLockEnabled = true;
        headingPID.reset();
        headingPID.setTargetPosition(targetHeading);
        Log.i("Drive", "Heading locked to: " + Math.toDegrees(targetHeading) + " degrees");
    }

    /**
     * Lock to a specific heading in degrees
     */
    public void lockHeadingDegrees(double headingDegrees) {
        lockHeading(Math.toRadians(headingDegrees));
    }

    /**
     * Lock to cardinal directions (0, 90, 180, 270 degrees)
     */
    public void lockToCardinal() {
        double currentHeadingDegrees = Math.toDegrees(getPose().getHeading());

        // Find nearest cardinal direction
        double nearest = Math.round(currentHeadingDegrees / 90.0) * 90.0;
        lockHeadingDegrees(nearest);

        Log.i("Drive", "Locked to nearest cardinal: " + nearest + " degrees");
    }

    /*
    * Auto Aim CLOSE
    */
    public void autoAimHeadingCLOSE() {
        lockHeading(shootHeadingCLOSE); // TODO: check if shootHeadingCLOSE is in radians or degrees

        Log.i("Drive", "Locked to close shooting heading: " + shootHeadingCLOSE + " degrees");
    }

    /*
     * Auto Aim FAR
     */
    public void autoAimHeadingFAR() {
        lockHeading(shootHeadingFAR); // TODO: check if shootHeadingFAR is in radians or degrees

        Log.i("Drive", "Locked to far shooting heading: " + shootHeadingFAR + " degrees");
    }


    /**
     * Disable heading lock
     */
    public void disableHeadingLock() {
        headingLockEnabled = false;
//        headingPID.reset();
        follower.breakFollowing();
        Log.i("Drive", "Heading lock disabled");
    }

    /**
     * Toggle heading lock on/off
     * If turning on, locks to current heading
     */
    public void toggleHeadingLock() {
        if (headingLockEnabled) {
            disableHeadingLock();
        } else {
            lockCurrentHeading();
        }
    }

    /**
     * Check if heading lock is enabled
     */
    public boolean isHeadingLockEnabled() {
        return headingLockEnabled;
    }

    /**
     * Get the target locked heading in radians
     */
    public double getTargetHeading() {
        return targetHeading;
    }

    /**
     * Get the heading error in radians (how far from target)
     */
    public double getHeadingError() {
        return normalizeAngle(targetHeading - getPose().getHeading());
    }

    /**
     * Check if robot is at target heading (within tolerance)
     */
    public boolean isAtTargetHeading() {
        return headingLockEnabled && Math.abs(headingError) < HEADING_TOLERANCE;
    }

    /**
     * Update heading PID constants for tuning
     */
    public void setHeadingPID(double kP, double kI, double kD) {
        headingPID.setCoefficients(new PIDFCoefficients(kP, kI, kD, HEADING_kF));
        Log.i("Drive", String.format("Heading PID updated: kP=%.3f, kI=%.3f, kD=%.3f", kP, kI, kD));
    }

    /**
     * Normalize angle to [-PI, PI] range
     */
    private double normalizeAngle(double angleRadians) {
        while (angleRadians > Math.PI) angleRadians -= 2 * Math.PI;
        while (angleRadians < -Math.PI) angleRadians += 2 * Math.PI;
        return angleRadians;
    }


    //------------------------AUTONOMOUS------------------------//

    public void followPath(com.pedropathing.paths.Path path, boolean holdEnd) {
        if (follower != null) {
            follower.followPath(path, holdEnd);
            currentMode = DriveMode.AUTONOMOUS;
            automatedDrive = true;
            Log.i("Drive", "Following path (holdEnd: " + holdEnd + ")");
        }
    }

    /**
     * Follow a Path (holds end by default)
     */
    public void followPath(com.pedropathing.paths.Path path) {
        followPath(path, true);
    }

    /**
     * Use for complex paths
     */
    public void followPath(com.pedropathing.paths.PathChain pathChain, boolean holdEnd) {
        if (follower != null) {
            follower.followPath(pathChain, holdEnd);
            currentMode = DriveMode.AUTONOMOUS;
            automatedDrive = true;
            Log.i("Drive", "Following path chain (holdEnd: " + holdEnd + ")");
        }
    }

    /**
     * Follow a PathChain (holds end by default)
     */
    public void followPath(com.pedropathing.paths.PathChain pathChain) {
        followPath(pathChain, true);
    }

    public com.pedropathing.paths.PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    /**
     * Check if the follower is busy following a path
     */
    public boolean isBusy() {
        return follower != null && follower.isBusy();
    }

    /**
     * Break following the current path and return to teleop control
     * Use for interrupting automated paths in teleop
     */
    public void breakFollowing() {
        if (follower != null) {
            follower.breakFollowing();
            automatedDrive = false;
            Log.i("Drive", "Breaking path following");
        }
    }

    /**
     * Stop automated drive and return to manual teleop control
     */
    public void stopAutomatedDrive() {
        if (automatedDrive) {
            follower.startTeleopDrive();
            automatedDrive = false;
            Log.i("Drive", "Stopped automated drive, returned to teleop");
        }
    }

    /**
     * Check if currently following an automated path
     */
    public boolean isAutomatedDriveActive() {
        return automatedDrive;
    }

    //------------------------POSE & LOCALIZATION------------------------//

    /**
     * Set the starting pose for Pedro Pathing
     */
    public void setStartingPose(Pose pose) {
        if (follower != null) {
            follower.setStartingPose(pose);
            Log.i("Drive", "Starting pose set to: " + pose.toString());
        }
    }

    /**
     * Set the starting pose using x, y, heading (in radians)
     */
    public void setStartingPose(double x, double y, double headingRadians) {
        setStartingPose(new Pose(x, y, headingRadians));
    }

    /**
     * Set pose
     */
    public void resetHeading() {
        Pose oldPose = getPoseEstimate();
        follower.setPose(new Pose(oldPose.getX(), oldPose.getY()));
    }

    public Pose getPoseEstimate() {
        return follower.getPose();
    }

    /**
     * Get the current robot pose from Pedro Pathing localization
     */
    public Pose getPose() {
        return follower != null ? follower.getPose() : new Pose(0, 0, 0);
    }

//    /**
//     * Get the current robot velocity from Pedro Pathing
//     */
//    public Pose getVelocity() {
//        return follower != null ? follower.getVelocity() : new Pose(0, 0, 0);
//    }

    /**
     * Get the Pedro Pathing follower (idk why i have, use only sometimes --> AdVaNCeD UsAGEs)
     */
    public Follower getFollower() {
        return follower;
    }

    //------------------------CONFIGURATION------------------------//

    /**
     * Set drive mode
     */
    public void setDriveMode(DriveMode mode) {
        this.currentMode = mode;
        Log.i("Drive", "Drive mode set to: " + mode);
    }

    /**
     * Get current drive mode
     */
    public DriveMode getDriveMode() {
        return currentMode;
    }

    /**
     * Enable or disable slow mode for teleop
     */
    public void enableSlowMode() {
        slowModeEnabled = true;
        setSlowModeMultiplier(0.5);
        Log.i("Drive", "Slow mode: " + "ON" + " (multiplier: " + slowModeMultiplier + ")");
    }

    /**
     * Toggle slow mode
     */
    public void disableSlowMode() {
        slowModeEnabled = false;
        setSlowModeMultiplier(1.0);
        Log.i("Drive", "Slow mode: " + "OFF");

    }

    /**
     * Check if slow mode is enabled
     */
    public boolean isSlowModeEnabled() {
        return slowModeEnabled;
    }

    /**
     * Set slow mode multiplier (0.0 to 1.0)
     */
    public void setSlowModeMultiplier(double multiplier) {
        slowModeMultiplier = MathFunctions.clamp(multiplier, 0d, 1d);
        Log.i("Drive", "Slow mode multiplier set to: " + this.slowModeMultiplier);
    }
    /**
     * Get slow mode multiplier
     */
    public double getSlowModeMultiplier() {
        return slowModeMultiplier;
    }

    /**
     * Increase slow mode strength by 0.25
     */
    public void increaseSlowModeStrength() {
        setSlowModeMultiplier(slowModeMultiplier + 0.25);
    }

    /**
     * Decrease slow mode strength by 0.25
     */
    public void decreaseSlowModeStrength() {
        setSlowModeMultiplier(slowModeMultiplier - 0.25);
    }

    /**
     * Set maximum autonomous path following power (0.0 to 1.0)
     */
    public void setMaxPower(double power) {
        if (follower != null) {
            follower.setMaxPower(Math.max(0.0, Math.min(1.0, power)));
            Log.i("Drive", "Max autonomous power set to: " + power);
        }
    }

    /**
     * Reset subsystem
     */
    public void reset() {
        slowModeEnabled = false;
        slowModeMultiplier = 0.5;
        automatedDrive = false;
        currentMode = DriveMode.TELEOP_ROBOT_CENTRIC;

        if (follower != null) {
            follower.setStartingPose(new Pose(0, 0, 0));
        }

        Log.i("Drive", "Reset complete");
    }


    @Override
    public void periodic() {
        // IMPORTANTIAL: Must call follower.update() once per loop
        if (follower != null) {
            follower.update();
        }

        // Update heading lock PID if enabled
        if (headingLockEnabled) {
            // Update PID coefficients from FTC Dashboard
            headingPID.setCoefficients(new PIDFCoefficients(HEADING_kP, HEADING_kI, HEADING_kD, HEADING_kF));

            // Calculate error
            double currentHeading = getPose().getHeading();
            headingError = normalizeAngle(targetHeading - currentHeading);

            // Update PID and get output
            headingPID.updateError(headingError);
            headingPower = headingPID.run();

            // Clamp power to reasonable range
            headingPower = Math.max(-0.5, Math.min(0.5, headingPower));
        } else {
            headingError = 0.0;
            headingPower = 0.0;
        }


        // If in teleop and automated path is done, return to manual control
        if (currentMode != DriveMode.AUTONOMOUS && automatedDrive && !follower.isBusy()) {
            stopAutomatedDrive();
        }
    }

}