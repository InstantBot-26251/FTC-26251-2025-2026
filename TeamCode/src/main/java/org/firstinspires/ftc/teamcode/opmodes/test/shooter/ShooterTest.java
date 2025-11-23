package org.firstinspires.ftc.teamcode.opmodes.test.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.globals.Enigma;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.ATVision;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Test: Shooter", group = "Test")
public class ShooterTest extends LinearOpMode {

    private Shooter shooter;
    private ATVision vision;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot
        Enigma robot = Enigma.getInstance();
        robot.init(hardwareMap, Alliance.BLUE);

        shooter = Shooter.getInstance();
        vision = ATVision.getInstance();

        shooter.onTeleopInit();
        vision.onTeleopInit();

        telemetry.addLine("Shooter Test OpMode");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A - Start Shooter");
        telemetry.addLine("B - Stop Shooter");
        telemetry.addLine("X - Auto Aim Hood (uses vision)");
        telemetry.addLine("Y - Reset Hood to Min");
        telemetry.addLine();
        telemetry.addLine("DPAD UP - Increase Hood Angle");
        telemetry.addLine("DPAD DOWN - Decrease Hood Angle");
        telemetry.addLine("DPAD LEFT - Decrease Shooter Power");
        telemetry.addLine("DPAD RIGHT - Increase Shooter Power");
        telemetry.update();

        waitForStart();

        double manualHoodPosition = 0.0;
        double shooterPower = 0.0;

        while (opModeIsActive()) {
            // Shooter control
            if (gamepad1.a) {
                shooter.startShooter();
            }
            if (gamepad1.b) {
                shooter.stopShooter();
            }

            // Hood control
            if (gamepad1.x) {
                boolean success = shooter.autoAimHood();
                if (success) {
                    manualHoodPosition = shooter.getHoodPosition();
                }
            }
            if (gamepad1.y) {
                manualHoodPosition = 0.0;
                shooter.setHoodPosition(manualHoodPosition);
            }

            // Manual hood adjustment
            if (gamepad1.dpad_up) {
                manualHoodPosition += 0.01;
                manualHoodPosition = Math.min(1.0, manualHoodPosition);
                shooter.setHoodPosition(manualHoodPosition);
            }
            if (gamepad1.dpad_down) {
                manualHoodPosition -= 0.01;
                manualHoodPosition = Math.max(0.0, manualHoodPosition);
                shooter.setHoodPosition(manualHoodPosition);
            }

            // Manual power adjustment
            if (gamepad1.dpad_right) {
                shooterPower += 0.05;
                shooterPower = Math.min(1.0, shooterPower);
                shooter.setShooterPower(shooterPower);
            }
            if (gamepad1.dpad_left) {
                shooterPower -= 0.05;
                shooterPower = Math.max(0.0, shooterPower);
                shooter.setShooterPower(shooterPower);
            }

            // Update periodic methods
            shooter.periodic();
            vision.periodic();

            // Telemetry
            telemetry.addData("Shooter Velocity", "%.0f ticks/sec", shooter.getShooterVelocity());
            telemetry.addData("Hood Position", "%.2f", shooter.getHoodPosition());
            telemetry.addData("Manual Hood Pos", "%.2f", manualHoodPosition);
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.addLine();
            telemetry.addData("Target Distance", "%.1f inches", shooter.getTargetDistance());
            telemetry.addData("AprilTag Detected", shooter.getTargetDistance() > 0 ? "YES" : "NO");
            telemetry.update();
        }

    }
}