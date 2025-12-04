package org.firstinspires.ftc.teamcode.commandbase.subsystems.drive;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drive extends SubsystemBase {

    private static final Drive INSTANCE = new Drive();
    public static Drive getInstance() {
        return INSTANCE;
    }

    private final Robot robot = Robot.getInstance();

    private Follower follower;

    private final ElapsedTime timer;

    public void initHardware(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    public void setMaxPower(double maxPower) {
        follower.setMaxPower(maxPower);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
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
        timer = new ElapsedTime();

        // Initialize PIDF Controller
        headingPID = new PIDFController(new PIDFCoefficients(HEADING_kP, HEADING_kI, HEADING_kD, HEADING_kF));

        Log.i("Drive", "DriveSubsystem initialized with Pedro Pathing 2.0.4");
    }


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

    /*
     * Auto Aim CLOSE
     */
    public void autoAimHeadingCLOSE() {
        lockHeading(shootHeadingCLOSE);
        Log.i("Drive", "Locked to close shooting heading: " + shootHeadingCLOSE + " degrees");
    }

    /*
     * Auto Aim FAR
     */
    public void autoAimHeadingFAR() {
        lockHeading(shootHeadingFAR);
        Log.i("Drive", "Locked to far shooting heading: " + shootHeadingFAR + " degrees");
    }

    public void breakFollowing() {
        follower.breakFollowing();
    }


    /**
     * Disable heading lock
     */
    public void disableHeadingLock() {
        headingLockEnabled = false;
        breakFollowing();
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
     * Normalize angle to [-PI, PI] range
     */
    private double normalizeAngle(double angleRadians) {
        while (angleRadians > Math.PI) angleRadians -= 2 * Math.PI;
        while (angleRadians < -Math.PI) angleRadians += 2 * Math.PI;
        return angleRadians;
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
     * Get the current robot pose
     */
    public Pose getPose() {
        return follower != null ? follower.getPose() : new Pose(0, 0, 0);
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
        // IMPORTANT: Must call follower.update() once per loop
        if (follower != null) {
            follower.update();
        }

        // Update heading lock PID if enabled
        if (headingLockEnabled) {
            // Update PID coefficients from FTC Dashboard
            headingPID.setCoefficients(new PIDFCoefficients(HEADING_kP, HEADING_kI, HEADING_kD, HEADING_kF));

            // Calculate error
            double currentHeading = getPose().getHeading();
            double headingError = normalizeAngle(targetHeading - currentHeading);

            // Update PID and get output
            headingPID.updateError(headingError);
            double headingPower = headingPID.run();

            // Clamp power to reasonable range
            headingPower = Math.max(-0.5, Math.min(0.5, headingPower));
        }

        // If in teleop and automated path is done, return to manual control
        if (currentMode != DriveMode.AUTONOMOUS && automatedDrive && !follower.isBusy()) {
            stopAutomatedDrive();
        }
    }

}