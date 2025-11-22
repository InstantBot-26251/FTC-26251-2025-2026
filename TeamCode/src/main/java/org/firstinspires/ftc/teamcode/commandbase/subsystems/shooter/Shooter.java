package org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.Enigma;
import org.firstinspires.ftc.teamcode.globals.RobotMap;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.ATVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Shooter extends SubsystemTemplate {

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private Servo hoodServo;

    private Telemetry telemetry;
    private ATVision vision;

    private static final Shooter INSTANCE = new Shooter();

    public static Shooter getInstance() {
        return INSTANCE;
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Enigma.getInstance().getTelemetry();
        vision = ATVision.getInstance();
        initHardware();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Enigma.getInstance().getTelemetry();
        vision = ATVision.getInstance();
        initHardware();
    }

    @Override
    public void onTestInit() {
        telemetry = Enigma.getInstance().getTelemetry();
        vision = ATVision.getInstance();
        initHardware();
    }

    @Override
    public void onAutonomousPeriodic() {
        updateTelemetry();
    }

    @Override
    public void onTeleopPeriodic() {
        updateTelemetry();
    }

    @Override
    public void onTestPeriodic() {
        updateTelemetry();
    }

    @Override
    public void onDisable() {
        stopShooter();
        setHoodPosition(HOOD_MIN_POSITION);
    }

    private void initHardware() {
        RobotMap map = RobotMap.getInstance();

        // Initialize shooter motors
        shooterMotor1 = map.SHOOTER_0;
        shooterMotor2 = map.SHOOTER_1;

        // Set motor directions
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize hood servo
        hoodServo = map.HOOD;
        hoodServo.setPosition(HOOD_MIN_POSITION);
    }

    /**
     * Set shooter motors to a specific power
     */
    public void setShooterPower(double power) {
        power = Math.max(0.0, Math.min(1.0, power)); // Clamp between 0 and 1
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    /**
     * Start the shooter at full power
     */
    public void startShooter() {
        setShooterPower(SHOOTER_MAX_POWER);
    }

    /**
     * Stop the shooter
     */
    public void stopShooter() {
        setShooterPower(SHOOTER_IDLE_POWER);
    }

    /**
     * Set hood to a specific position
     */
    public void setHoodPosition(double position) {
        position = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, position));
        hoodServo.setPosition(position);
    }

    /**
     * Get the current distance to the AprilTag using vision
     */
    public double getTargetDistance() {
        ArrayList<AprilTagDetection> detections = vision.getDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            if (detection.ftcPose != null) {
                return detection.ftcPose.range; // Distance in inches
            }
        }

        return -1.0; // No detection
    }

    /**
     * Calculate the ideal hood position based on distance using linear interpolation
     * This uses an empirically-determined lookup table
     */
    private double calculateHoodAngle(double distance) {
        // Handle edge cases
        if (distance <= HOOD_LOOKUP_TABLE[0][0]) {
            return HOOD_LOOKUP_TABLE[0][1]; // Closest distance
        }
        if (distance >= HOOD_LOOKUP_TABLE[HOOD_LOOKUP_TABLE.length - 1][0]) {
            return HOOD_LOOKUP_TABLE[HOOD_LOOKUP_TABLE.length - 1][1]; // Farthest distance
        }

        // Find the two points to interpolate between
        for (int i = 0; i < HOOD_LOOKUP_TABLE.length - 1; i++) {
            double dist1 = HOOD_LOOKUP_TABLE[i][0];
            double dist2 = HOOD_LOOKUP_TABLE[i + 1][0];

            if (distance >= dist1 && distance <= dist2) {
                // Linear interpolation between the two points
                double pos1 = HOOD_LOOKUP_TABLE[i][1];
                double pos2 = HOOD_LOOKUP_TABLE[i + 1][1];

                double t = (distance - dist1) / (dist2 - dist1);
                return pos1 + t * (pos2 - pos1);
            }
        }

        // Fallback (should never reach here)
        return HOOD_MIN_POSITION;
    }

    /**
     * Automatically adjust hood angle based on AprilTag distance
     */
    public boolean autoAimHood() {
        double distance = getTargetDistance();

        if (distance > 0) {
            double hoodPosition = calculateHoodAngle(distance);
            setHoodPosition(hoodPosition);

            telemetry.addData("Target Distance", "%.1f inches", distance);
            telemetry.addData("Hood Position", "%.2f", hoodPosition);

            return true;
        } else {
            telemetry.addData("Auto Aim", "No target detected");
            return false;
        }
    }

    /**
     * Get current shooter velocity (average of both motors) - ticks per second
     */
    public double getShooterVelocity() {
        return (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0;
    }

    /**
     * Check if shooter is at target velocity
     */
    public boolean isAtSpeed(double targetVelocity, double tolerance) {
        double currentVelocity = getShooterVelocity();
        return Math.abs(currentVelocity - targetVelocity) < tolerance; // acceptable error margin
    }

    /**
     * Get current hood position
     */
    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    private void updateTelemetry() {
        telemetry.addData("Shooter Velocity", "%.0f ticks/sec", getShooterVelocity());
        telemetry.addData("Hood Position", "%.2f", getHoodPosition());
        telemetry.addData("Target Distance", "%.1f inches", getTargetDistance());
    }
}