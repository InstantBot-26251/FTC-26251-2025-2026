package org.firstinspires.ftc.teamcode.solverslib.opmodes.test.drive;

import static org.firstinspires.ftc.teamcode.solverslib.globals.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.solverslib.commandbase.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;


@TeleOp(name = "Test: Drive Pedro", group = "Test")
public class DriveTestPedro extends OpMode {
    private Robot robot;

    private GamepadEx driver;
    @Override
    public void init() {
        robot = Robot.getInstance();
        robot.teleopInit(telemetry, hardwareMap);

        telemetry.addLine("Drive Test OpMode Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick: Forward/Strafe");
        telemetry.addLine("  Right Stick X: Turn");
        telemetry.addLine("  A: Toggle Slow Mode");
        telemetry.addLine("  B: Toggle Heading Lock");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Field-centric drive
        new TeleopDriveCommand(
                () -> applyResponseCurve(driver.getLeftY(), DRIVE_SENSITIVITY),
                () -> -applyResponseCurve(driver.getLeftX(), DRIVE_SENSITIVITY),
                () -> -applyResponseCurve(driver.getRightX(), ROTATIONAL_SENSITIVITY) * ROTATION_DAMPEN);

        // Toggle slow mode with A button
        if (gamepad1.a && !lastA) {
            robot.drive.disableSlowMode();
        }
        lastA = gamepad1.a;

        // Toggle heading lock with B button
        if (gamepad1.b && !lastB) {
            robot.drive.toggleHeadingLock();
        }
        lastB = gamepad1.b;

        // Update drive subsystem
        robot.drive.periodic();

        // Telemetry
        telemetry.addData("Drive Mode", "Field Centric");
        telemetry.addData("Slow Mode", robot.drive.isSlowModeEnabled() ? "ON" : "OFF");
        telemetry.addData("Heading Lock", robot.drive.isHeadingLockEnabled() ? "ON" : "OFF");
        telemetry.addLine();
        telemetry.addData("Position X", "%.2f", robot.drive.getPose().getX());
        telemetry.addData("Position Y", "%.2f", robot.drive.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robot.drive.getPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Forward", "%.2f", driver.getLeftY());
        telemetry.addData("Strafe", "%.2f", driver.getLeftX());
        telemetry.addData("Turn", "%.2f", driver.getRightX());
        telemetry.update();
    }

    private double applyResponseCurve(double input, double scale) {
        // Limit Input to 1 (MAX) and -1 (MIN)
        input = Math.max(-1, Math.min(1, input));

        // Apply Response Curve
        return Math.signum(input) * Math.pow(Math.abs(input), scale);
    }

    // Button state tracking for toggling stuff
    private boolean lastA = false;
    private boolean lastB = false;
    }

