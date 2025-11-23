package org.firstinspires.ftc.teamcode.opmodes.test.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.globals.Enigma;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.ATVision;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Test: Hood Tuning", group = "Test")
public class HoodTuningTest extends LinearOpMode {

    private Shooter shooter;
    private ATVision vision;

    // Data collection for lookup table
    private List<DataPoint> collectedData = new ArrayList<>();

    private static class DataPoint {
        double distance;
        double hoodPosition;
        int successfulShots;
        int totalShots;

        DataPoint(double distance, double hoodPosition) {
            this.distance = distance;
            this.hoodPosition = hoodPosition;
            this.successfulShots = 0;
            this.totalShots = 0;
        }
    }

    private DataPoint currentDataPoint = null;
    private double currentHoodPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot
        Enigma robot = Enigma.getInstance();
        robot.init(hardwareMap, Alliance.BLUE);

        shooter = Shooter.getInstance();
        vision = ATVision.getInstance();

        shooter.onTeleopInit();
        vision.onTeleopInit();

        telemetry.addLine("Hood Lookup Table Tuning OpMode");
        telemetry.addLine();
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Position robot at known distance");
        telemetry.addLine("2. Use DPAD to adjust hood angle");
        telemetry.addLine("3. Press A to start shooter");
        telemetry.addLine("4. Shoot and record success");
        telemetry.addLine("5. Press X to save data point");
        telemetry.addLine("6. Repeat for all distances");
        telemetry.addLine();
        telemetry.addLine("Recommended test distances:");
        telemetry.addLine("30, 36, 42, 48, 54, 60, 66, 72,");
        telemetry.addLine("78, 84, 90, 96, 102, 108, 114, 116 inches");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A - Start/Stop Shooter");
        telemetry.addLine("B - Mark Shot as SUCCESS");
        telemetry.addLine("Y - Mark Shot as MISS");
        telemetry.addLine("X - Save Current Data Point");
        telemetry.addLine();
        telemetry.addLine("DPAD UP/DOWN - Adjust Hood (fine)");
        telemetry.addLine("DPAD LEFT/RIGHT - Adjust Hood (coarse)");
        telemetry.addLine();
        telemetry.addLine("START - Print Collected Data");
        telemetry.addLine("BACK - Clear All Data");
        telemetry.update();

        waitForStart();

        boolean shooterRunning = false;
        boolean aPressed = false, bPressed = false, yPressed = false, xPressed = false;
        boolean upPressed = false, downPressed = false, leftPressed = false, rightPressed = false;
        boolean startPressed = false, backPressed = false;

        while (opModeIsActive()) {
            // Shooter control
            if (gamepad1.a && !aPressed) {
                shooterRunning = !shooterRunning;
                if (shooterRunning) {
                    shooter.startShooter();
                } else {
                    shooter.stopShooter();
                }
            }
            aPressed = gamepad1.a;

            // Record success
            if (gamepad1.b && !bPressed) {
                recordShot(true);
            }
            bPressed = gamepad1.b;

            // Record miss
            if (gamepad1.y && !yPressed) {
                recordShot(false);
            }
            yPressed = gamepad1.y;

            // Save data point
            if (gamepad1.x && !xPressed) {
                saveDataPoint();
            }
            xPressed = gamepad1.x;

            // Hood adjustment - fine control
            if (gamepad1.dpad_up && !upPressed) {
                currentHoodPosition += 0.01;
                currentHoodPosition = Math.min(1.0, currentHoodPosition);
                shooter.setHoodPosition(currentHoodPosition);
            }
            upPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !downPressed) {
                currentHoodPosition -= 0.01;
                currentHoodPosition = Math.max(0.0, currentHoodPosition);
                shooter.setHoodPosition(currentHoodPosition);
            }
            downPressed = gamepad1.dpad_down;

            // Hood adjustment - coarse control
            if (gamepad1.dpad_right && !rightPressed) {
                currentHoodPosition += 0.05;
                currentHoodPosition = Math.min(1.0, currentHoodPosition);
                shooter.setHoodPosition(currentHoodPosition);
            }
            rightPressed = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !leftPressed) {
                currentHoodPosition -= 0.05;
                currentHoodPosition = Math.max(0.0, currentHoodPosition);
                shooter.setHoodPosition(currentHoodPosition);
            }
            leftPressed = gamepad1.dpad_left;

            // Print data
            if (gamepad1.start && !startPressed) {
                printCollectedData();
            }
            startPressed = gamepad1.start;

            // Clear data
            if (gamepad1.back && !backPressed) {
                collectedData.clear();
                currentDataPoint = null;
                telemetry.addLine(">>> DATA CLEARED <<<");
                telemetry.update();
                sleep(1000);
            }
            backPressed = gamepad1.back;

            // Update subsystems
            shooter.periodic();
            vision.periodic();

            // Display current status
            telemetry.addData("Shooter", shooterRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Velocity", "%.0f ticks/sec", shooter.getShooterVelocity());
            telemetry.addLine();

            double distance = shooter.getTargetDistance();
            telemetry.addData("Distance (Vision)", distance > 0 ?
                    String.format("%.1f inches", distance) : "NOT DETECTED");
            telemetry.addLine();

            telemetry.addData("Hood Position", "%.3f", currentHoodPosition);
            telemetry.addLine();

            // Current data point status
            if (currentDataPoint != null) {
                telemetry.addLine("=== CURRENT DATA POINT ===");
                telemetry.addData("Distance", "%.1f inches", currentDataPoint.distance);
                telemetry.addData("Hood Position", "%.3f", currentDataPoint.hoodPosition);
                telemetry.addData("Shots", "%d / %d successful",
                        currentDataPoint.successfulShots, currentDataPoint.totalShots);
                if (currentDataPoint.totalShots > 0) {
                    double accuracy = (currentDataPoint.successfulShots * 100.0) / currentDataPoint.totalShots;
                    telemetry.addData("Accuracy", "%.1f%%", accuracy);
                }
            } else {
                telemetry.addLine("No active data point");
                telemetry.addLine("Record shots to create one");
            }
            telemetry.addLine();

            // Summary of collected data
            telemetry.addData("Data Points Saved", collectedData.size());
            if (!collectedData.isEmpty()) {
                telemetry.addLine("Press START to view all data");
            }

            telemetry.update();
        }

    }

    private void recordShot(boolean success) {
        double distance = shooter.getTargetDistance();

        if (distance <= 0) {
            telemetry.addLine(">>> ERROR: No distance detected! <<<");
            telemetry.update();
            return;
        }

        // Create new data point if needed
        if (currentDataPoint == null ||
                Math.abs(currentDataPoint.distance - distance) > 2.0 ||
                Math.abs(currentDataPoint.hoodPosition - currentHoodPosition) > 0.02) {

            currentDataPoint = new DataPoint(distance, currentHoodPosition);
        }

        // Record the shot
        currentDataPoint.totalShots++;
        if (success) {
            currentDataPoint.successfulShots++;
        }

        telemetry.addLine(success ? ">>> SHOT: SUCCESS <<<" : ">>> SHOT: MISS <<<");
        telemetry.update();
    }

    private void saveDataPoint() {
        if (currentDataPoint == null || currentDataPoint.totalShots == 0) {
            telemetry.addLine(">>> ERROR: No shots recorded! <<<");
            telemetry.update();
            sleep(1000);
            return;
        }

        // Only save if accuracy is good (at least 80%)
        double accuracy = (currentDataPoint.successfulShots * 100.0) / currentDataPoint.totalShots;

        if (accuracy < 80.0) {
            telemetry.addLine(">>> WARNING: Low accuracy! <<<");
            telemetry.addData("Current Accuracy", "%.1f%%", accuracy);
            telemetry.addLine("Save anyway? Press X again to confirm");
            telemetry.update();
            sleep(2000);

            // Check for confirmation
            if (!gamepad1.x) {
                return;
            }
        }

        collectedData.add(currentDataPoint);

        telemetry.addLine(">>> DATA POINT SAVED <<<");
        telemetry.addData("Distance", "%.1f inches", currentDataPoint.distance);
        telemetry.addData("Hood Position", "%.3f", currentDataPoint.hoodPosition);
        telemetry.addData("Accuracy", "%.1f%%", accuracy);
        telemetry.update();

        currentDataPoint = null;
        sleep(1500);
    }

    private void printCollectedData() {
        telemetry.clear();
        telemetry.addLine("=== COLLECTED DATA ===");
        telemetry.addLine();

        if (collectedData.isEmpty()) {
            telemetry.addLine("No data collected yet");
        } else {
            telemetry.addLine("Copy this into your Shooter class:");
            telemetry.addLine("private static final double[][] HOOD_LOOKUP_TABLE = {");

            for (int i = 0; i < collectedData.size(); i++) {
                DataPoint dp = collectedData.get(i);
                double accuracy = (dp.successfulShots * 100.0) / dp.totalShots;

                String line = String.format("    {%.1f, %.3f}%s  // %.1f%% accuracy (%d/%d)",
                        dp.distance,
                        dp.hoodPosition,
                        i < collectedData.size() - 1 ? "," : " ",
                        accuracy,
                        dp.successfulShots,
                        dp.totalShots);

                telemetry.addLine(line);
            }

            telemetry.addLine("};");
            telemetry.addLine();
            telemetry.addLine("Press BACK to return");
        }

        telemetry.update();

        // Wait for back button
        while (opModeIsActive() && !gamepad1.back) {
            sleep(100);
        }
    }
}