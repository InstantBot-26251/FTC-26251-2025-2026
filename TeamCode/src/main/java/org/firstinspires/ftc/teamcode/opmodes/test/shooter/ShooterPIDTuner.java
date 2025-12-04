package org.firstinspires.ftc.teamcode.opmodes.test.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;

@Config
@TeleOp(name = "Shooter Tuning", group = "Test")
public class ShooterPIDTuner extends OpMode {

    public static double TARGET_VELOCITY = 1500.0;

    private Shooter shooter;
    private boolean shooterRunning = false;

    private boolean prevA = false;
    private boolean prevB = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = Shooter.getInstance();
        shooter.initHardware(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // Start shooter at target velocity
        boolean currentA = gamepad1.a;
        if (currentA && !prevA) {
            shooterRunning = true;
            shooter.setShooterVelocityTicks(TARGET_VELOCITY);
        }
        prevA = currentA;

        // Stop shooter
        boolean currentB = gamepad1.b;
        if (currentB && !prevB) {
            shooterRunning = false;
            shooter.stopShooter();
        }
        prevB = currentB;

        // Update target velocity if shooter is running
        if (shooterRunning) {
            shooter.setShooterVelocityTicks(TARGET_VELOCITY);
        }

        // Display telemetry
        double currentVel = shooter.getShooterVelocity();
        double error = TARGET_VELOCITY - currentVel;
        double percentError = TARGET_VELOCITY > 0 ? (error / TARGET_VELOCITY) * 100 : 0;

        telemetry.addLine("=== SHOOTER STATUS ===");
        telemetry.addData("Running", shooterRunning ? "YES" : "NO");
        telemetry.addData("Target Velocity", "%.0f ticks/sec", TARGET_VELOCITY);
        telemetry.addData("Current Velocity", "%.0f ticks/sec", currentVel);
        telemetry.addData("Error", "%.0f ticks/sec (%.1f%%)", error, percentError);
        telemetry.addData("Ready", shooter.isReadyToShoot() ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stopShooter();
    }
}