package org.firstinspires.ftc.teamcode.opmodes.test.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.globals.Enigma;
import org.firstinspires.ftc.teamcode.util.Alliance;


@TeleOp(name = "Test: Drive Pedro", group = "Test")
public class DriveTestPedro extends OpMode {
    private DriveSubsystem drive;
    private Enigma robot;

    @Override
    public void init() {
        Enigma robot = Enigma.getInstance();
        robot.init(hardwareMap, Alliance.BLUE);

        drive = DriveSubsystem.getInstance();

        drive.onTeleopInit();

        telemetry.addLine("Drive Test OpMode Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick: Forward/Strafe");
        telemetry.addLine("  Right Stick X: Turn");
        telemetry.addLine("  A: Toggle Slow Mode");
        telemetry.addLine("  B: Toggle Heading Lock");
        telemetry.addLine("Ready to drive in Field Centric mode!");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forwardSpeed = -gamepad1.left_stick_y;  // Forward/backward (inverted)
        double strafeSpeed = gamepad1.left_stick_x;    // Left/right strafe
        double turnSpeed = gamepad1.right_stick_x;     // Rotation

        // Field-centric drive
        drive.driveFieldCentric(forwardSpeed, strafeSpeed, turnSpeed);

        // Toggle slow mode with A button
        if (gamepad1.a && !lastA) {
            drive.toggleSlowMode();
        }
        lastA = gamepad1.a;

        // Toggle heading lock with B button
        if (gamepad1.b && !lastB) {
            drive.toggleHeadingLock();
        }
        lastB = gamepad1.b;

        // Update drive subsystem
        drive.periodic();

        // Telemetry
        telemetry.addData("Drive Mode", "Field Centric");
        telemetry.addData("Slow Mode", drive.isSlowModeEnabled() ? "ON" : "OFF");
        telemetry.addData("Heading Lock", drive.isHeadingLockEnabled() ? "ON" : "OFF");
        telemetry.addLine();
        telemetry.addData("Position X", "%.2f", drive.getPose().getX());
        telemetry.addData("Position Y", "%.2f", drive.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(drive.getPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Forward", "%.2f", forwardSpeed);
        telemetry.addData("Strafe", "%.2f", strafeSpeed);
        telemetry.addData("Turn", "%.2f", turnSpeed);
        telemetry.update();
    }

    // Button state tracking for toggle functionality
    private boolean lastA = false;
    private boolean lastB = false;
    }

