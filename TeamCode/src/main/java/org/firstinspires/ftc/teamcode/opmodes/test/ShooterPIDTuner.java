package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.RollingAverage;


@TeleOp(name = "Shooter: PID Tuner", group = "Testing")
public class ShooterPIDTuner extends OpMode {

    private Shooter shooter;

    // Tunable gains
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;

    // Gain selection
    private enum Gain { KP, KI, KD, KF }
    private Gain currentGain = Gain.KP;

    // Adjustment steps
    private double[] adjustmentSteps = {0.0001, 0.001, 0.01, 0.1};
    private int currentStepIndex = 1; // Start with 0.001

    // Target velocities (ticks/sec) - adjust based on your robot
    private static final double LOW_VELOCITY = 1000.0;
    private static final double MEDIUM_VELOCITY = 1500.0;
    private static final double HIGH_VELOCITY = 2000.0;
    private double targetVelocity = MEDIUM_VELOCITY;

    // Control state
    private boolean shooterActive = false;

    // Button debouncing
    private ElapsedTime buttonTimer = new ElapsedTime();
    private static final double BUTTON_COOLDOWN = 0.15;

    // Performance metrics
    private ElapsedTime performanceTimer = new ElapsedTime();
    private RollingAverage errorAverage = new RollingAverage(20);
    private double maxOvershoot = 0.0;
    private double settlingTime = 0.0;
    private boolean settled = false;
    private double riseTime = 0.0;
    private boolean reachedTarget = false;

    @Override
    public void init() {
        shooter = Shooter.getInstance();

        // Load current PIDF values as starting point
         kP = ShooterConstants.SHOOTER_PIDF_COEFFICIENTS.p;
         kI = ShooterConstants.SHOOTER_PIDF_COEFFICIENTS.i;
         kD = ShooterConstants.SHOOTER_PIDF_COEFFICIENTS.d;
         kF = ShooterConstants.SHOOTER_PIDF_COEFFICIENTS.f;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "PID Tuning");
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.onTeleopInit();
        buttonTimer.reset();
        performanceTimer.reset();
    }

    @Override
    public void loop() {
        handleControls();

        if (shooterActive) {
            applyPIDFGains();
            updatePerformanceMetrics();
        }

        displayTelemetry();
    }

    private void handleControls() {
        if (buttonTimer.seconds() < BUTTON_COOLDOWN) {
            return;
        }

        boolean buttonPressed = false;

        // Adjust current gain
        if (gamepad1.dpad_up) {
            adjustGain(getCurrentStepSize());
            buttonPressed = true;
        } else if (gamepad1.dpad_down) {
            adjustGain(-getCurrentStepSize());
            buttonPressed = true;
        }

        // Switch between gains
        if (gamepad1.right_bumper) {
            currentGain = Gain.values()[(currentGain.ordinal() + 1) % Gain.values().length];
            buttonPressed = true;
        } else if (gamepad1.left_bumper) {
            int newIndex = currentGain.ordinal() - 1;
            if (newIndex < 0) newIndex = Gain.values().length - 1;
            currentGain = Gain.values()[newIndex];
            buttonPressed = true;
        }

        // Change step size
        if (gamepad1.right_trigger > 0.5) {
            currentStepIndex = Math.min(adjustmentSteps.length - 1, currentStepIndex + 1);
            buttonPressed = true;
        } else if (gamepad1.left_trigger > 0.5) {
            currentStepIndex = Math.max(0, currentStepIndex - 1);
            buttonPressed = true;
        }

        // Velocity presets
        if (gamepad1.a) {
            targetVelocity = LOW_VELOCITY;
            resetMetrics();
            buttonPressed = true;
        } else if (gamepad1.b) {
            targetVelocity = MEDIUM_VELOCITY;
            resetMetrics();
            buttonPressed = true;
        } else if (gamepad1.y) {
            targetVelocity = HIGH_VELOCITY;
            resetMetrics();
            buttonPressed = true;
        }

        // Toggle shooter
        if (gamepad1.x) {
            shooterActive = !shooterActive;
            if (shooterActive) {
                shooter.setShooterVelocityTicks(targetVelocity);
                resetMetrics();
            } else {
                shooter.stopShooter();
            }
            buttonPressed = true;
        }

        // Reset to current constants
        if (gamepad1.start) {
             kP = ShooterConstants.SHOOTER_PIDF_COEFFICIENTS.p;
             kI = ShooterConstants.SHOOTER_PIDF_COEFFICIENTS.i;
             kD = ShooterConstants.SHOOTER_PIDF_COEFFICIENTS.d;
             kF = ShooterConstants.SHOOTER_PIDF_COEFFICIENTS.f;
            resetMetrics();
            buttonPressed = true;
        }

        // Zero all gains
        if (gamepad1.back) {
            kP = 0.0;
            kI = 0.0;
            kD = 0.0;
            kF = 0.0;
            resetMetrics();
            buttonPressed = true;
        }

        if (buttonPressed) {
            buttonTimer.reset();
        }
    }

    private void adjustGain(double delta) {
        switch (currentGain) {
            case KP:
                kP = Math.max(0, kP + delta);
                break;
            case KI:
                kI = Math.max(0, kI + delta);
                break;
            case KD:
                kD = Math.max(0, kD + delta);
                break;
            case KF:
                kF = Math.max(0, kF + delta);
                break;
        }
        resetMetrics();
    }

    private double getCurrentStepSize() {
        return adjustmentSteps[currentStepIndex];
    }

    private void applyPIDFGains() {
        // Apply gains directly to the shooter's PIDF controller
        shooter.updatePIDFGains(kP, kI, kD, kF);
    }

    private void updatePerformanceMetrics() {
        double currentVel = shooter.getShooterVelocity();
        double error = targetVelocity - currentVel;

        // Track error
        errorAverage.add(Math.abs(error));

        // Track overshoot
        if (currentVel > targetVelocity) {
            double overshoot = ((currentVel - targetVelocity) / targetVelocity) * 100.0;
            maxOvershoot = Math.max(maxOvershoot, overshoot);
        }

        // Track rise time (time to reach 90% of target)
        if (!reachedTarget && currentVel >= targetVelocity * 0.9) {
            riseTime = performanceTimer.seconds();
            reachedTarget = true;
        }

        // Track settling time (within 2% of target)
        if (reachedTarget && !settled) {
            if (Math.abs(error) < targetVelocity * 0.02) {
                if (performanceTimer.seconds() > settlingTime + 0.5) {
                    settlingTime = performanceTimer.seconds();
                    settled = true;
                }
            } else {
                settlingTime = performanceTimer.seconds();
            }
        }
    }

    private void resetMetrics() {
        errorAverage.clear();
        maxOvershoot = 0.0;
        settlingTime = 0.0;
        settled = false;
        riseTime = 0.0;
        reachedTarget = false;
        performanceTimer.reset();
    }

    private void displayTelemetry() {
        telemetry.addData("=== SHOOTER PID TUNER ===", "");
        telemetry.addData("", "");

        // Current gains
        telemetry.addData("--- PIDF Gains ---", "");
        telemetry.addData(currentGain == Gain.KP ? ">>> Kp <<<" : "Kp", "%.6f", kP);
        telemetry.addData(currentGain == Gain.KI ? ">>> Ki <<<" : "Ki", "%.6f", kI);
        telemetry.addData(currentGain == Gain.KD ? ">>> Kd <<<" : "Kd", "%.6f", kD);
        telemetry.addData(currentGain == Gain.KF ? ">>> Kf <<<" : "Kf", "%.6f", kF);
        telemetry.addData("Step Size", "%.6f", getCurrentStepSize());

        telemetry.addData("", "");

        // Shooter status
        telemetry.addData("--- Shooter Status ---", "");
        telemetry.addData("Active", shooterActive ? "YES" : "NO");
        telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocity);
        telemetry.addData("Current Velocity", "%.0f ticks/sec", shooter.getShooterVelocity());
        telemetry.addData("Error", "%.0f ticks/sec", targetVelocity - shooter.getShooterVelocity());
        telemetry.addData("Error %", "%.1f%%",
                Math.abs((targetVelocity - shooter.getShooterVelocity()) / targetVelocity * 100.0));

        telemetry.addData("", "");

        // Performance metrics
        telemetry.addData("--- Performance ---", "");
        telemetry.addData("Avg Error", "%.1f ticks/sec", errorAverage.getAverage());
        telemetry.addData("Max Overshoot", "%.1f%%", maxOvershoot);
        telemetry.addData("Rise Time", reachedTarget ? "%.2f sec" : "N/A", riseTime);
        telemetry.addData("Settling Time", settled ? "%.2f sec" : "N/A", settlingTime);
        telemetry.addData("Stable", settled ? "YES" : "NO");

        telemetry.addData("", "");

        // Controls reminder
        telemetry.addData("--- Controls ---", "");
        telemetry.addData("DPAD UP/DOWN", "Adjust " + currentGain.name());
        telemetry.addData("RB/LB", "Switch Gain");
        telemetry.addData("RT/LT", "Change Step Size");
        telemetry.addData("A/B/Y", "Low/Med/High Target");
        telemetry.addData("X", "Toggle Shooter");
        telemetry.addData("START", "Reset to Constants");
        telemetry.addData("BACK", "Zero All Gains");

        telemetry.addData("", "");

        // Tuning tips
        String tip = getTuningTip();
        if (!tip.isEmpty()) {
            telemetry.addData("--- Tip ---", "");
            telemetry.addData("", tip);
        }

        telemetry.update();
    }

    private String getTuningTip() {
        if (!shooterActive) {
            return "Press X to start shooter";
        }

        double error = Math.abs(targetVelocity - shooter.getShooterVelocity());
        double errorPercent = (error / targetVelocity) * 100.0;

        if (kP == 0.0 && kI == 0.0 && kD == 0.0 && kF == 0.0) {
            return "Start by increasing Kp until oscillation occurs";
        } else if (maxOvershoot > 15.0) {
            return "High overshoot! Reduce Kp or increase Kd";
        } else if (errorPercent > 5.0 && settled) {
            return "Steady-state error detected. Try increasing Ki";
        } else if (!reachedTarget && performanceTimer.seconds() > 3.0) {
            return "Slow response. Increase Kp or Kf";
        } else if (errorAverage.getAverage() < 20.0 && settled) {
            return "Good tuning! Record these values";
        }

        return "";
    }

    @Override
    public void stop() {
        shooter.stopShooter();
        shooter.onDisable();

        // Print final gains to console
        telemetry.addData("=== FINAL GAINS ===", "");
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.update();
    }
}