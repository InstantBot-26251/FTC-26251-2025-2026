//package org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.ilc;
//
//import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.ilc.ILCConstants.*;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.Range;
//import com.seattlesolvers.solverslib.command.SubsystemBase;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.ATVision;
//
//import java.util.Arrays;
//
//public class InertialLaunchCoreILC extends SubsystemBase {
//
//    private DcMotorEx ilcAlpha; // Left
//    private DcMotorEx ilcBeta; // Right
////    private Servo hoodServo; // TODO: No hood for now
//
//    public static int vel = 0;
//
//    private Telemetry telemetry;
//    private ATVision vision;
//
//    // Empirically determined flywheel velocities at various distances
//    private final InterpolatingLUT VELOCITY_LOOKUP_TABLE = new InterpolatingLUT(
//            Arrays.asList(10.0,   14.0,   18.0,   22.0,   26.0,   30.0,   34.0,   38.0,   42.0),
//            Arrays.asList(1200.0, 1320.0, 1450.0, 1580.0, 1700.0, 1830.0, 1950.0, 2075.0, 2200.0)
//    );
//
//    private boolean activeVelocityControl = false;
//    private double targetVelocityTicks = 0.0;
//    private boolean activeControl = false;
//
//
//    public InertialLaunchCoreILC(HardwareMap hardwareMap) {
//        VELOCITY_LOOKUP_TABLE.createLUT();
//
//        initHardware(hardwareMap);
//
//    }
//
//
//    public void initHardware(HardwareMap hardwareMap) {
//        // Initialize ILC
//        ilcAlpha = hardwareMap.get(DcMotorEx.class, "ilcL");
//        ilcBeta = hardwareMap.get(DcMotorEx.class, "ilcR");
//
//        ilcBeta.setDirection(DcMotor.Direction.REVERSE);
//
//        ilcAlpha.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        ilcAlpha.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        ilcBeta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
////        ilcBeta.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//
////        // Initialize hood servo
////        hoodServo = map.HOOD;
////        hoodServo.setPosition(HOOD_MIN_POSITION); //TODO: No hood for now
//
//        // Make sure velocity control is off on init
//        activeVelocityControl = false;
//        targetVelocityTicks = 0.0;
//    }
//
//    public void periodic() {
////        updateVelocityControl();
//        updateTelemetry();
//    }
//
////    public void setShooter(double vel, boolean setActiveControl) {
////        shooterController.setSetPoint(Math.min(VELOCITY_LOOKUP_TABLE.get(vel), SHOOTER_MAX_VELOCITY));
////        targetVelocityTicks = vel;
////        activeControl = setActiveControl;
////    }
//
//    public void setArbalestVelocity(int vel) {
//        ilcAlpha.setVelocity(vel);
//        ilcBeta.setPower(ilcAlpha.getPower());
//    }
//
//    public double getVelError() {
//        return Math.abs(ilcAlpha.getVelocity() - vel);
//    }
//
//    /**
//     * Set shooter motors to a specific power
//     */
////    public void setShooterPower(double power) {
////        activeVelocityControl = false;
////        power = Math.max(0.0, Math.min(1.0, power));
////        shooterMotor1.setPower(power);
////        shooterMotor2.setPower(power);
////    }
//
////    /**
////     * Start the shooter at full power
////     */
////    public void startShooter() {
////        setShooterPower(SHOOTER_MAX_POWER);
////    }
////
////    /**
////     * Stop the shooter
////     */
////    public void stopShooter() {
////        setShooterPower(SHOOTER_IDLE_POWER);
////    }
//
////    /**
////     * Set hood to a specific position
////     */
////    public void setHoodPosition(double position) {
////        position = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, position));
////        hoodServo.setPosition(position);
////    } //TODO: No hood for now
//
//    /**
//     * Check if shooter is ready to fire
//     */
//    public boolean isReadyToShoot() {
//        if (!activeVelocityControl) {
//            return false;
//        }
//
//        if (ilcAlpha.getTargetPosition() == ilcAlpha.getCurrentPosition() || getVelError() <= 20) {
//            return true;
//        } else {
//            return false;
//        }
//    }
//
//    /**
//     * Increase target velocity by a fixed amount (for manual tuning)
//     */
////    public void increaseVelocity(double delta) {
////        if (activeVelocityControl) {
////            setShooterVelocityTicks(targetVelocityTicks + delta);
////        }
////    }
//
//    /**
//     * Decrease target velocity by a fixed amount (for manual tuning)
//     */
////    public void decreaseVelocity(double delta) {
////        if (activeVelocityControl) {
////            setShooterVelocityTicks(Math.max(0, targetVelocityTicks - delta));
////        }
////    }
//
////    /**
////     * Increment hood position for manual adjustment
////     */
////    public void incrementHood(double delta) {
////        double newPosition = getHoodPosition() + delta;
////        setHoodPosition(newPosition);
////    } //TODO: No hood for now
//
////    /**
////     * Reset hood to home position
////     */
////    public void resetHood() {
////        setHoodPosition(HOOD_MIN_POSITION);
////    } //TODO: No hood for now
//
//    /**
//     * Get shooter status for telemetry
//     */
//    public String getILCStatus() {
//        if (!activeVelocityControl) {
//            return "MANUAL";
//        } else if (isReadyToShoot()) {
//            return "READY";
//        } else {
//            return "SPINNING_UP";
//        }
//    }
//
//    public void stopILC() {
//        ilcAlpha.setPower(0);
//        ilcBeta.setPower(0);
//    }
//    /**
//     * Get the current distance to the AprilTag using vision
//     */
//    public double getTargetDistance() {
//        return vision.getDistance();
//    }
//
//    /**
//     * Get current shooter velocity (average of both motors) in ticks/sec.
//     */
//    public double getILCVelocity() {
//        return (ilcAlpha.getVelocity() + ilcBeta.getVelocity()) / 2.0;
//    }
//
//    /**
//     * Check if shooter is at target velocity (absolute error < tolerance).
//     */
//    public boolean isAtSpeed(double targetVelocity, double tolerance) {
//        double currentVelocity = getILCVelocity();
//        return Math.abs(currentVelocity - targetVelocity) < tolerance;
//    }
//
////    /**
////     * Calculate the ideal hood position based on distance using linear interpolation
////     */
////    public double calculateHoodAngle(double distance) {
////        if (distance <= HOOD_LOOKUP_TABLE[0][0]) {
////            return HOOD_LOOKUP_TABLE[0][1];
////        }
////        if (distance >= HOOD_LOOKUP_TABLE[HOOD_LOOKUP_TABLE.length - 1][0]) {
////            return HOOD_LOOKUP_TABLE[HOOD_LOOKUP_TABLE.length - 1][1];
////        }
////
////        for (int i = 0; i < HOOD_LOOKUP_TABLE.length - 1; i++) {
////            double dist1 = HOOD_LOOKUP_TABLE[i][0];
////            double dist2 = HOOD_LOOKUP_TABLE[i + 1][0];
////
////            if (distance >= dist1 && distance <= dist2) {
////                double pos1 = HOOD_LOOKUP_TABLE[i][1];
////                double pos2 = HOOD_LOOKUP_TABLE[i + 1][1];
////                double t = (distance - dist1) / (dist2 - dist1);
////                return pos1 + t * (pos2 - pos1);
////            }
////        }
////
////        return HOOD_MIN_POSITION;
////    }
//
////    /**
////     * Automatically adjust hood angle based on AprilTag distance
////     */
////    public boolean autoAimHood() {
////        double distance = getTargetDistance();
////
////        if (distance > 0) {
////            double hoodPosition = calculateHoodAngle(distance);
////            setHoodPosition(hoodPosition);
////
////            telemetry.addData("Target Distance", "%.1f inches", distance);
////            telemetry.addData("Hood Position", "%.2f", hoodPosition);
////
////            return true;
////        } else {
////            telemetry.addData("Auto Aim", "No target detected");
////            return false;
////        }
////    } //TODO: No hood for now
//
//    /**
//     * Automatically aim both hood and velocity based on AprilTag distance
//     * sike Auto sets only velocity because no hood for now
//     */
//    public boolean autoAim() {
//        double distance = getTargetDistance();
//
//        if (distance > 0) {
////            double hoodPosition = calculateHoodAngle(distance);
////            setHoodPosition(hoodPosition);
//
//            setILCVelocityForDistance(distance);
//
//            telemetry.addData("Auto Aim", "Active");
//            telemetry.addData("Target Distance", "%.1f inches", distance);
//            telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocityTicks);
//
//            return true;
//        } else {
//            telemetry.addData("Auto Aim", "No target detected");
//            return false;
//        }
//    }
//
////    /**
////     * Set shooter velocity directly in ticks/sec and enable closed-loop PIDF.
////     */
////    public void setShooterVelocityTicks(double velTicksPerSec) {
////        targetVelocityTicks = velTicksPerSec;
////        shooterController.setSetPoint(velTicksPerSec);
////        activeVelocityControl = true;
////    }
//
//    /**
//     * Get desired flywheel velocity (ticks/sec) for a given distance (inches) using LUT.
//     */
//    private double getTargetVelocityForDistance(double distanceInches) {
//        return VELOCITY_LOOKUP_TABLE.get(distanceInches);
//    }
//
//    /**
//     * Set shooter velocity based on distance (inches) using LUT.
//     */
//    public void setILCVelocityForDistance(double distanceInches) {
//        double vel = getTargetVelocityForDistance(distanceInches);
//        vel = Range.clip(vel, 0.0, ARBALEST_MAX_POWER);
//        setArbalestVelocity((int) vel);
//    }
//
//    /**
//     * Update flywheel closed-loop control each cycle.
//     */
////    private void updateVelocityControl() {
////        if (!activeVelocityControl) {
////            return;
////        }
////
////        double currentVel = getShooterVelocity();
////
////        // The FlywheelPIDController now handles both feedforward and feedback
////        double output = shooterController.calculate(currentVel);
////
////        // Clamp output to motor range
////        output = Range.clip(output, -1.0, 1.0);
////
////        shooterMotor1.setPower(output);
////        shooterMotor2.setPower(output);
////    }
////
////    /**
////     * Get current hood position
////     */
////    public double getHoodPosition() {
////        return hoodServo.getPosition();
////    }
//
//    private void updateTelemetry() {
//        telemetry.addData("=== ILC ===", "");
//        telemetry.addData("Status", getILCStatus());
//        telemetry.addData("Current Velocity", "%.0f ticks/sec", getILCVelocity());
//
//        if (activeVelocityControl) {
//            telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocityTicks);
//            telemetry.addData("Velocity Error", "%.0f ticks/sec",
//                    getVelError());
//        }
//
////        telemetry.addData("Hood Position", "%.3f", getHoodPosition());
//
//        double distance = getTargetDistance();
//        if (distance > 0) {
//            telemetry.addData("Target Distance", "%.1f inches", distance);
//        } else {
//            telemetry.addData("Target Distance", "NO TARGET");
//        }
//
//        telemetry.addData("Motor 1 Velocity", "%.0f", ilcAlpha.getVelocity());
//        telemetry.addData("Motor 2 Velocity", "%.0f", ilcBeta.getVelocity());
//    }
//
////    /**
////     * Update PIDF dynamically - for tuning
////     */
////    public void updatePIDFGains(double p, double i, double d, double f) {
////        shooterController.setP(p);
////        shooterController.setI(i);
////        shooterController.setD(d);
////        shooterController.setF(f);
////    }
//
//}