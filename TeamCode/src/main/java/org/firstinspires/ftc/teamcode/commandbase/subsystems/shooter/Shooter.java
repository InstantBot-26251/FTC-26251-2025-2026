package org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.RobotMap;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.ATVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;

public class Shooter extends SubsystemBase {

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
//    private Servo hoodServo; //TODO: No hood for now

    private Telemetry telemetry;
    private ATVision vision;

    // Empirically determined flywheel velocities at various distances
    private final InterpolatingLUT VELOCITY_LOOKUP_TABLE = new InterpolatingLUT(
            Arrays.asList(10.0,   14.0,   18.0,   22.0,   26.0,   30.0,   34.0,   38.0,   42.0),
            Arrays.asList(1200.0, 1320.0, 1450.0, 1580.0, 1700.0, 1830.0, 1950.0, 2075.0, 2200.0)
    );

    private PIDFController shooterController = new PIDFController(SHOOTER_PIDF_COEFFICIENTS);
    private boolean activeVelocityControl = false;
    private double targetVelocityTicks = 0.0;
    private boolean activeControl = false;

    private static final Shooter INSTANCE = new Shooter();

    public static Shooter getInstance() {
        return INSTANCE;
    }

    public Shooter() {
        VELOCITY_LOOKUP_TABLE.createLUT();
        shooterController.setTolerance(SHOOTER_VEL_TOLERANCE);
    }


    public void initHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.vision = ATVision.getInstance();

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

//        // Initialize hood servo
//        hoodServo = map.HOOD;
//        hoodServo.setPosition(HOOD_MIN_POSITION); //TODO: No hood for now

        // Make sure velocity control is off on init
        activeVelocityControl = false;
        targetVelocityTicks = 0.0;
    }

    @Override
    public void periodic() {
        updateVelocityControl();
        updateTelemetry();
    }

    public void setShooter(double vel, boolean setActiveControl) {
        shooterController.setSetPoint(Math.min(VELOCITY_LOOKUP_TABLE.get(vel), SHOOTER_MAX_VELOCITY));
        targetVelocityTicks = vel;
        activeControl = setActiveControl;
    }

    /**
     * Set shooter motors to a specific power
     */
    public void setShooterPower(double power) {
        activeVelocityControl = false;
        power = Math.max(0.0, Math.min(1.0, power));
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

//    /**
//     * Set hood to a specific position
//     */
//    public void setHoodPosition(double position) {
//        position = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, position));
//        hoodServo.setPosition(position);
//    } //TODO: No hood for now

    /**
     * Check if shooter is ready to fire
     */
    public boolean isReadyToShoot() {
        if (!activeVelocityControl) {
            return false;
        }

        return shooterController.atSetPoint();
    }

    /**
     * Increase target velocity by a fixed amount (for manual tuning)
     */
    public void increaseVelocity(double delta) {
        if (activeVelocityControl) {
            setShooterVelocityTicks(targetVelocityTicks + delta);
        }
    }

    /**
     * Decrease target velocity by a fixed amount (for manual tuning)
     */
    public void decreaseVelocity(double delta) {
        if (activeVelocityControl) {
            setShooterVelocityTicks(Math.max(0, targetVelocityTicks - delta));
        }
    }

//    /**
//     * Increment hood position for manual adjustment
//     */
//    public void incrementHood(double delta) {
//        double newPosition = getHoodPosition() + delta;
//        setHoodPosition(newPosition);
//    } //TODO: No hood for now

//    /**
//     * Reset hood to home position
//     */
//    public void resetHood() {
//        setHoodPosition(HOOD_MIN_POSITION);
//    } //TODO: No hood for now

    /**
     * Get shooter status for telemetry
     */
    public String getShooterStatus() {
        if (!activeVelocityControl) {
            return "MANUAL";
        } else if (isReadyToShoot()) {
            return "READY";
        } else {
            return "SPINNING_UP";
        }
    }

    /**
     * Get the current distance to the AprilTag using vision
     */
    public double getTargetDistance() {
        ArrayList<AprilTagDetection> detections = vision.getDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            if (detection.ftcPose != null) {
                return detection.ftcPose.range;
            }
        }

        return -1.0;
    }

    /**
     * Get current shooter velocity (average of both motors) in ticks/sec.
     */
    public double getShooterVelocity() {
        return (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0;
    }

    /**
     * Check if shooter is at target velocity (absolute error < tolerance).
     */
    public boolean isAtSpeed(double targetVelocity, double tolerance) {
        double currentVelocity = getShooterVelocity();
        return Math.abs(currentVelocity - targetVelocity) < tolerance;
    }

//    /**
//     * Calculate the ideal hood position based on distance using linear interpolation
//     */
//    public double calculateHoodAngle(double distance) {
//        if (distance <= HOOD_LOOKUP_TABLE[0][0]) {
//            return HOOD_LOOKUP_TABLE[0][1];
//        }
//        if (distance >= HOOD_LOOKUP_TABLE[HOOD_LOOKUP_TABLE.length - 1][0]) {
//            return HOOD_LOOKUP_TABLE[HOOD_LOOKUP_TABLE.length - 1][1];
//        }
//
//        for (int i = 0; i < HOOD_LOOKUP_TABLE.length - 1; i++) {
//            double dist1 = HOOD_LOOKUP_TABLE[i][0];
//            double dist2 = HOOD_LOOKUP_TABLE[i + 1][0];
//
//            if (distance >= dist1 && distance <= dist2) {
//                double pos1 = HOOD_LOOKUP_TABLE[i][1];
//                double pos2 = HOOD_LOOKUP_TABLE[i + 1][1];
//                double t = (distance - dist1) / (dist2 - dist1);
//                return pos1 + t * (pos2 - pos1);
//            }
//        }
//
//        return HOOD_MIN_POSITION;
//    }

//    /**
//     * Automatically adjust hood angle based on AprilTag distance
//     */
//    public boolean autoAimHood() {
//        double distance = getTargetDistance();
//
//        if (distance > 0) {
//            double hoodPosition = calculateHoodAngle(distance);
//            setHoodPosition(hoodPosition);
//
//            telemetry.addData("Target Distance", "%.1f inches", distance);
//            telemetry.addData("Hood Position", "%.2f", hoodPosition);
//
//            return true;
//        } else {
//            telemetry.addData("Auto Aim", "No target detected");
//            return false;
//        }
//    } //TODO: No hood for now

    /**
     * Emergency stop - immediately stop shooter
     */
    public void emergencyStop() {
        activeVelocityControl = false;
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        telemetry.addData("EMERGENCY STOP", "Shooter halted");
    } // NEVER USE


    /**
     * Automatically aim both hood and velocity based on AprilTag distance
     * sike Auto sets only velocity because no hood for now
     */
    public boolean autoAim() {
        double distance = getTargetDistance();

        if (distance > 0) {
//            double hoodPosition = calculateHoodAngle(distance);
//            setHoodPosition(hoodPosition);

            setShooterVelocityForDistance(distance);

            telemetry.addData("Auto Aim", "Active");
            telemetry.addData("Target Distance", "%.1f inches", distance);
            telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocityTicks);

            return true;
        } else {
            telemetry.addData("Auto Aim", "No target detected");
            return false;
        }
    }

    /**
     * Set shooter velocity directly in ticks/sec and enable closed-loop PIDF.
     */
    public void setShooterVelocityTicks(double velTicksPerSec) {
        targetVelocityTicks = velTicksPerSec;
        shooterController.setSetPoint(velTicksPerSec);
        activeVelocityControl = true;
    }

    /**
     * Get desired flywheel velocity (ticks/sec) for a given distance (inches) using LUT.
     */
    private double getTargetVelocityForDistance(double distanceInches) {
        return VELOCITY_LOOKUP_TABLE.get(distanceInches);
    }

    /**
     * Set shooter velocity based on distance (inches) using LUT.
     */
    public void setShooterVelocityForDistance(double distanceInches) {
        double vel = getTargetVelocityForDistance(distanceInches);
        vel = Range.clip(vel, 0.0, SHOOTER_MAX_VELOCITY);
        setShooterVelocityTicks(vel);
    }

    /**
     * Update flywheel closed-loop control each cycle.
     */
    private void updateVelocityControl() {
        if (!activeVelocityControl) {
            return;
        }

        double currentVel = getShooterVelocity();
        double output = shooterController.calculate(currentVel);

        output = Range.clip(output, -1.0, 1.0);

        shooterMotor1.setPower(output);
        shooterMotor2.setPower(output);
    }
//
//    /**
//     * Get current hood position
//     */
//    public double getHoodPosition() {
//        return hoodServo.getPosition();
//    }

    private void updateTelemetry() {
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Status", getShooterStatus());
        telemetry.addData("Current Velocity", "%.0f ticks/sec", getShooterVelocity());

        if (activeVelocityControl) {
            telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocityTicks);
            telemetry.addData("Velocity Error", "%.0f ticks/sec",
                    targetVelocityTicks - getShooterVelocity());
        }

//        telemetry.addData("Hood Position", "%.3f", getHoodPosition());

        double distance = getTargetDistance();
        if (distance > 0) {
            telemetry.addData("Target Distance", "%.1f inches", distance);
        } else {
            telemetry.addData("Target Distance", "NO TARGET");
        }

        telemetry.addData("Motor 1 Velocity", "%.0f", shooterMotor1.getVelocity());
        telemetry.addData("Motor 2 Velocity", "%.0f", shooterMotor2.getVelocity());
    }

    /**
     * Update PIDF gains dynamically - for tuning
     */
    public void updatePIDFGains(double p, double i, double d, double f) {
        shooterController.setP(p);
        shooterController.setI(i);
        shooterController.setD(d);
        shooterController.setF(f);
    }

}