package org.firstinspires.ftc.teamcode.subsystems.drive;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Enigma;

public class DriveSubsystem extends SubsystemBase {

    private Follower follower;

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

    public DriveSubsystem() {
        // This is the official Pedro 2.0.4 initialization method
        follower = Constants.createFollower(Enigma.getInstance().getHardwareMap());

        // Set default starting pose
        follower.setStartingPose(new Pose(0, 0, 0));

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

    //------------------------PEDRO PATHING AUTONOMOUS------------------------//

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
    public void setSlowMode(boolean enabled) {
        this.slowModeEnabled = enabled;
        Log.i("Drive", "Slow mode: " + (enabled ? "ON" : "OFF") + " (multiplier: " + slowModeMultiplier + ")");
    }

    /**
     * Toggle slow mode
     */
    public void toggleSlowMode() {
        setSlowMode(!slowModeEnabled);
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
        this.slowModeMultiplier = Math.max(0.0, Math.min(1.0, multiplier));
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

        // If in teleop and automated path is done, return to manual control
        if (currentMode != DriveMode.AUTONOMOUS && automatedDrive && !follower.isBusy()) {
            stopAutomatedDrive();
        }
    }
}