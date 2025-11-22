package org.firstinspires.ftc.teamcode.opmodes.test.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.ATVision;

import java.util.ArrayList;
import java.util.Locale;

@TeleOp(name = "Test: Shooter Data Collection", group = "Test")
public class ShooterDataCollector extends OpMode {

    private Shooter shooter;
    private ATVision vision;

    // Data collection
    private ArrayList<DataPoint> collectedData = new ArrayList<>();
    private double currentTestVelocity = 1500.0;
    private double currentTestHoodPosition = 0.3;

    // Control parameters
    private static final double VELOCITY_STEP = 50.0;
    private static final double HOOD_STEP = 0.01;
    private static final double VELOCITY_FINE_STEP = 10.0;
    private static final double HOOD_FINE_STEP = 0.002;

    // Button debouncing
    private ElapsedTime buttonTimer = new ElapsedTime();
    private static final double BUTTON_COOLDOWN = 0.2;

    // Velocity monitoring
    private ElapsedTime velocityStableTimer = new ElapsedTime();
    private boolean velocityStable = false;

    @Override
    public void init() {
        shooter = Shooter.getInstance();
        vision = ATVision.getInstance();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "See code comments");
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.onTeleopInit();
        vision.onTeleopInit();
        buttonTimer.reset();
        velocityStableTimer.reset();
    }

    @Override
    public void loop() {
        handleControls();
        updateVelocityMonitoring();
        displayTelemetry();
    }

    private void handleControls() {
        if (buttonTimer.seconds() < BUTTON_COOLDOWN) {
            return; // Still in cooldown
        }

        boolean buttonPressed = false;

        // Velocity control (coarse)
        if (gamepad1.dpad_up) {
            currentTestVelocity += VELOCITY_STEP;
            buttonPressed = true;
        } else if (gamepad1.dpad_down) {
            currentTestVelocity = Math.max(0, currentTestVelocity - VELOCITY_STEP);
            buttonPressed = true;
        }

        // Velocity control (fine)
        if (gamepad1.right_bumper && gamepad1.dpad_up) {
            currentTestVelocity += VELOCITY_FINE_STEP;
            buttonPressed = true;
        } else if (gamepad1.right_bumper && gamepad1.dpad_down) {
            currentTestVelocity = Math.max(0, currentTestVelocity - VELOCITY_FINE_STEP);
            buttonPressed = true;
        }

        // Hood control (coarse)
        if (gamepad1.dpad_right) {
            currentTestHoodPosition += HOOD_STEP;
            buttonPressed = true;
        } else if (gamepad1.dpad_left) {
            currentTestHoodPosition -= HOOD_STEP;
            buttonPressed = true;
        }

        // Hood control (fine)
        if (gamepad1.right_bumper && gamepad1.dpad_right) {
            currentTestHoodPosition += HOOD_FINE_STEP;
            buttonPressed = true;
        } else if (gamepad1.right_bumper && gamepad1.dpad_left) {
            currentTestHoodPosition -= HOOD_FINE_STEP;
            buttonPressed = true;
        }

        // Apply settings
        if (gamepad1.x) {
            shooter.setShooterVelocityTicks(currentTestVelocity);
            shooter.setHoodPosition(currentTestHoodPosition);
            buttonPressed = true;
        }

        // Stop shooter
        if (gamepad1.b) {
            shooter.stopShooter();
            buttonPressed = true;
        }

        // Save data point
        if (gamepad1.a && velocityStable) {
            saveDataPoint();
            buttonPressed = true;
        }

        // Clear all data
        if (gamepad1.back) {
            collectedData.clear();
            buttonPressed = true;
        }

        if (buttonPressed) {
            buttonTimer.reset();
        }
    }

    private void updateVelocityMonitoring() {
        double currentVel = shooter.getShooterVelocity();
        double error = Math.abs(currentVel - currentTestVelocity);

        if (error < 30.0 && currentTestVelocity > 0) { // Within 30 ticks/sec
            if (velocityStableTimer.seconds() > 1.0) { // Stable for 1 second
                velocityStable = true;
            }
        } else {
            velocityStable = false;
            velocityStableTimer.reset();
        }
    }

    private void saveDataPoint() {
        double distance = shooter.getTargetDistance();

        if (distance > 0) {
            DataPoint point = new DataPoint(
                    distance,
                    currentTestVelocity,
                    currentTestHoodPosition,
                    shooter.getShooterVelocity()
            );
            collectedData.add(point);

            // Sort by distance
            collectedData.sort((a, b) -> Double.compare(a.distance, b.distance));

            telemetry.speak("Data point saved");
        } else {
            telemetry.speak("No target detected");
        }
    }

    private void displayTelemetry() {
        telemetry.addData("=== SHOOTER DATA COLLECTOR ===", "");
        telemetry.addData("", "");

        // Current settings
        telemetry.addData("--- Current Settings ---", "");
        telemetry.addData("Target Velocity", "%.0f ticks/sec", currentTestVelocity);
        telemetry.addData("Actual Velocity", "%.0f ticks/sec", shooter.getShooterVelocity());
        telemetry.addData("Hood Position", "%.4f", currentTestHoodPosition);

        double distance = shooter.getTargetDistance();
        if (distance > 0) {
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Ready to Save", velocityStable ? "YES (Press A)" : "NO - Stabilizing...");
        } else {
            telemetry.addData("Distance", "NO TARGET DETECTED");
            telemetry.addData("Ready to Save", "NO");
        }

        telemetry.addData("", "");

        // Controls
        telemetry.addData("--- Controls ---", "");
        telemetry.addData("DPAD UP/DOWN", "Velocity ±%.0f", VELOCITY_STEP);
        telemetry.addData("RB + DPAD UP/DOWN", "Velocity ±%.0f (fine)", VELOCITY_FINE_STEP);
        telemetry.addData("DPAD LEFT/RIGHT", "Hood ±%.3f", HOOD_STEP);
        telemetry.addData("RB + DPAD L/R", "Hood ±%.4f (fine)", HOOD_FINE_STEP);
        telemetry.addData("X", "Apply Settings & Spin Up");
        telemetry.addData("B", "Stop Shooter");
        telemetry.addData("A", "Save Data Point (when stable)");
        telemetry.addData("BACK", "Clear All Data");

        telemetry.addData("", "");

        // Collected data
        telemetry.addData("--- Collected Data Points ---", "(%d)", collectedData.size());

        if (collectedData.isEmpty()) {
            telemetry.addData("", "No data collected yet");
        } else {
            telemetry.addData("", "Copy these into your constants file:");
            telemetry.addData("", "");

            // Velocity LUT
            StringBuilder distances = new StringBuilder("Arrays.asList(");
            StringBuilder velocities = new StringBuilder("Arrays.asList(");

            for (int i = 0; i < collectedData.size(); i++) {
                DataPoint point = collectedData.get(i);
                distances.append(String.format(Locale.US, "%.1f", point.distance));
                velocities.append(String.format(Locale.US, "%.0f", point.targetVelocity));

                if (i < collectedData.size() - 1) {
                    distances.append(", ");
                    velocities.append(", ");
                }
            }
            distances.append(")");
            velocities.append(")");

            telemetry.addData("Distances", distances.toString());
            telemetry.addData("Velocities", velocities.toString());
            telemetry.addData("", "");

            // Hood LUT
            telemetry.addData("HOOD_LOOKUP_TABLE = {", "");
            for (int i = 0; i < collectedData.size(); i++) {
                DataPoint point = collectedData.get(i);
                telemetry.addData("", "    {%.1f, %.4f}%s",
                        point.distance,
                        point.hoodPosition,
                        i < collectedData.size() - 1 ? "," : "");
            }
            telemetry.addData("", "}");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopShooter();
        shooter.onDisable();
    }

    /**
     * Data point class to store collected measurements
     */
    private static class DataPoint {
        double distance;
        double targetVelocity;
        double hoodPosition;
        double actualVelocity;

        DataPoint(double distance, double targetVelocity, double hoodPosition, double actualVelocity) {
            this.distance = distance;
            this.targetVelocity = targetVelocity;
            this.hoodPosition = hoodPosition;
            this.actualVelocity = actualVelocity;
        }
    }
}