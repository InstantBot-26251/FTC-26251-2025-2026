package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveConstants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;

public class AimLaunch extends CommandBase {

    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;

    private final ElapsedTime timer;
    private final ElapsedTime stageTimer;

    private final double goalX;
    private final double goalY;

    private double kP = 3.0;
    private int stage = 0;

    // Stage constants
    private static final int STAGE_AIM_DRIVETRAIN = 0;
    private static final int STAGE_ANGLE_HOOD = 1;
    private static final int STAGE_SPINUP = 2;
    private static final int STAGE_TRANSFER = 3;
    private static final int STAGE_COMPLETE = 4;

    // Tolerances and timeouts
    private static final double HEADING_TOLERANCE = Math.toRadians(2.0);
    private static final double HOOD_TOLERANCE = 0.02; // Hood position tolerance
    private static final double SPINUP_TIMEOUT = 1.5; // seconds
    private static final double TRANSFER_TIMEOUT = 2.0; // seconds

    private double targetDistance = 0.0;
    private double targetHeading = 0.0;
    private boolean impossibleShot = false;
    private boolean usingOdometry = false;

    public AimLaunch() {
        drive = Drive.getInstance();
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();

        this.timer = new ElapsedTime();
        this.stageTimer = new ElapsedTime();

        goalX = GOAL_POSE_X;
        goalY = GOAL_POSE_Y;

        addRequirements(drive, shooter, intake);
    }

    @Override
    public void initialize() {
        stage = STAGE_AIM_DRIVETRAIN;
        timer.reset();
        stageTimer.reset();
        impossibleShot = false;
        usingOdometry = false;

        // Stop intake and prepare for shooting
        intake.setIntake(IntakeState.STOP);
        shooter.setShooter(shooter.getShooterVelocity(), true);

        RobotLog.aa("AimCommand", "Initialized staged auto-aim sequence");
    }

    @Override
    public void execute() {
        Pose pose = drive.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        // Calculate target heading based on current position (always updated)
        targetHeading = Math.atan2(goalY - robotY, goalX - robotX);
        double headingError = angleWrap(targetHeading - robotHeading);

        // Calculate distance using vision first, fallback to O T O S
        targetDistance = getTargetDistance(pose);

        switch (stage) {
            case STAGE_AIM_DRIVETRAIN:
                executeAimDrivetrain(headingError);
                break;

            case STAGE_ANGLE_HOOD:
                executeAngleHood(headingError);
                break;

            case STAGE_SPINUP:
                executeSpinup(headingError);
                break;

            case STAGE_TRANSFER:
                executeTransfer(headingError);
                break;

            case STAGE_COMPLETE:
                // Do nothing, waiting for prev command to finish
                break;
        }
    }


     // Get target distance - prioritize vision, fallback to odometry
    private double getTargetDistance(Pose pose) {
        // Try to get vision distance first
        double visionDistance = shooter.getTargetDistance();

        if (visionDistance > 0) {
            // Valid vision reading
            usingOdometry = false;
            return visionDistance;
        } else {
            // Vision failed, use odometry
            usingOdometry = true;
            double dx = goalX - pose.getX();
            double dy = goalY - pose.getY();
            return Math.sqrt(dx * dx + dy * dy);
        }
    }


     // Apply heading lock - calculates turn power to maintain aim at goal
    private double calculateHeadingLock(double headingError) {
        double turn = kP * headingError;
        // Clamp turn speed so robot doesn't explode
        return Math.max(-0.5, Math.min(0.5, turn));
    }

    private void executeAimDrivetrain(double headingError) {
        double turn = calculateHeadingLock(headingError);

        drive.driveFieldCentric(0.0, 0.0, turn);

        RobotLog.aa("AimCommand", String.format("Stage 0: Aiming - error=%.3f rad, turn=%.3f, dist=%.2f, using_odom=%b",
                headingError, turn, targetDistance, usingOdometry));

        // Check if aligned
        if (Math.abs(headingError) < HEADING_TOLERANCE) {
            advanceStage();
        }
    }

    private void executeAngleHood(double headingError) {
        // Apply heading lock to maintain aim while setting hood
        double turn = calculateHeadingLock(headingError);
        drive.driveFieldCentric(0.0, 0.0, turn);

        // Continuously update shooter velocity based on current distance
        shooter.setShooterVelocityForDistance(targetDistance);

        // Continuously update hood angle based on current distance
        double targetHoodPosition = shooter.calculateHoodAngle(targetDistance);
        shooter.setHoodPosition(targetHoodPosition);

        RobotLog.aa("AimCommand", String.format("Stage 1: Angling hood - dist=%.2f, target_pos=%.3f, current_pos=%.3f, heading_err=%.3f, using_odom=%b",
                targetDistance, targetHoodPosition, shooter.getHoodPosition(), headingError, usingOdometry));

        // Check if hood is at target angle (within tolerance)
        if (Math.abs(shooter.getHoodPosition() - targetHoodPosition) < HOOD_TOLERANCE) {
            advanceStage(); // prob don't need this but just good to have for now
        }

        // Also advance if we've been trying for too long (servo might be stuck)
        if (stageTimer.seconds() > 1.0) {
            RobotLog.aa("AimCommand", "Hood timeout, advancing anyway");
            advanceStage();
        }
    }

    private void executeSpinup(double headingError) {
        // Apply heading lock to maintain aim while spinning up
        double turn = calculateHeadingLock(headingError);
        drive.driveFieldCentric(0.0, 0.0, turn);

        // Continuously update shooter settings based on current distance
        shooter.setShooterVelocityForDistance(targetDistance);
        double targetHoodPosition = shooter.calculateHoodAngle(targetDistance);
        shooter.setHoodPosition(targetHoodPosition);

        boolean atSpeed = shooter.isReadyToShoot();
        boolean timeout = stageTimer.seconds() > SPINUP_TIMEOUT;

        RobotLog.aa("AimCommand", String.format("Stage 2: Spinning up - time=%.2f, ready=%b, vel=%.0f, dist=%.2f, heading_err=%.3f, using_odom=%b",
                stageTimer.seconds(), atSpeed, shooter.getShooterVelocity(), targetDistance, headingError, usingOdometry));

        // Advance if at speed or timeout
        if (atSpeed || timeout) {
            if (timeout) {
                RobotLog.aa("AimCommand", "Spinup timeout, advancing anyway");
            }
            advanceStage();
        }
    }

    private void executeTransfer(double headingError) {
        // Apply heading lock to maintain aim while transferring
        double turn = calculateHeadingLock(headingError);
        drive.driveFieldCentric(0.0, 0.0, turn);

        // Continue updating hood and velocity even during transfer
        shooter.setShooterVelocityForDistance(targetDistance);
        double targetHoodPosition = shooter.calculateHoodAngle(targetDistance);
        shooter.setHoodPosition(targetHoodPosition);

        // Run transfer to feed ring to shooter
        intake.setIntake(IntakeState.TRANSFER);

        RobotLog.aa("AimCommand", String.format("Stage 3: Transferring - time=%.2f, dist=%.2f, heading_err=%.3f, using_odom=%b",
                stageTimer.seconds(), targetDistance, headingError, usingOdometry));

        // Check if transfer is complete (sensor no longer detects ring) or timeout
        boolean transferComplete = !intake.transferFull(); // Ring has left
        boolean timeout = stageTimer.seconds() > TRANSFER_TIMEOUT;

        if (transferComplete || timeout) {
            if (timeout) {
                RobotLog.aa("AimCommand", "Transfer timeout");
            }
            advanceStage();
        }
    }

    private void advanceStage() {
        stage++;
        stageTimer.reset();
        RobotLog.aa("AimCommand", "Advanced to stage " + stage);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all subsystems
        drive.driveFieldCentric(0.0, 0.0, 0.0);
        shooter.stopShooter();
        intake.setIntake(IntakeState.STOP);

        // Reset detection count for next time
        intake.resetDetectionCount();

        RobotLog.aa("AimCommand", String.format("Ended at stage %d (interrupted=%b, time=%.2f, impossible=%b)",
                stage, interrupted, timer.seconds(), impossibleShot));

    }

    @Override
    public boolean isFinished() {
        return impossibleShot || (stage >= STAGE_COMPLETE);
    }

    public int getStage() {
        return stage;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public boolean isUsingOdometry() {
        return usingOdometry;
    }

    private static double angleWrap(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}