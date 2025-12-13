package org.firstinspires.ftc.teamcode.subsystemspromax.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands.ShootSequenceCommand;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc.InertialLaunchCore;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends OpMode {
    // Subsystems
    private Intake intake;
    private InertialLaunchCore ilc;
    private Follower follower;

    // Commands
    private ShootSequenceCommand shootCommand;

    // Hardware
    private List<LynxModule> hubs;

    // Alliance
    private Alliance alliance = Alliance.BLUE;

    // Button state tracking (for toggling)
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastLeftBumper = false;
    private boolean lastX = false;

    @Override
    public void init() {
        // Setup telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize subsystems
        intake = new Intake(hardwareMap);
        ilc = new InertialLaunchCore(hardwareMap, intake);
        follower = Constants.createFollower(hardwareMap);

        // Initialize commands
        shootCommand = new ShootSequenceCommand(ilc, intake);

        // Initialize hardware
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Set starting pose
        follower.setPose(new Pose(0, 0, Math.toRadians(180)));
        follower.startTeleopDrive();

        // Reset subsystems
        intake.setIDLE();
        intake.resumeTransferControl();
        ilc.forceIdle();

        telemetry.addLine("TeleOp Initialized");
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            alliance = Alliance.BLUE;
        }

        if (gamepad1.b) {
            alliance = Alliance.RED;
        }

        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clear bulk cache FIRST
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // Update subsystems
        follower.update();
        ilc.periodic();
        intake.periodic();

        // Run command scheduler
        CommandScheduler.getInstance().run();

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x * 0.9;

        // Apply response curve
        drive = applyResponseCurve(drive, 1.1);
        strafe = applyResponseCurve(strafe, 1.1);
        turn = applyResponseCurve(turn, 2.0);

        // Slow mode with right trigger
        if (gamepad1.right_trigger > 0.1) {
            drive *= 0.3;
            strafe *= 0.3;
            turn *= 0.3;
        }

        follower.setTeleOpDrive(drive, strafe, turn, false);

        // B button - Toggle intake (pressed and released detection)
        if (gamepad2.b && !lastB) {
            if (intake.getIntakeState() == IntakeState.IDLE) {
                intake.setIntake(IntakeState.INTAKING);
            } else {
                intake.setIDLE();
            }
        }
        lastB = gamepad2.b;

        // X button - Stop intake
        if (gamepad2.x && !lastX) {
            intake.setIDLE();
        }
        lastX = gamepad2.x;

        // Y button - Reverse (hold)
        if (gamepad2.y) {
            intake.setIntake(IntakeState.REVERSE);
        } else if (!gamepad2.y && intake.getIntakeState() == IntakeState.REVERSE) {
            // When Y is released, go back to idle
            intake.setIDLE();
        }

        // A button - Full shoot sequence
        if (gamepad2.a && !lastA) {
            shootCommand.schedule();
        }
        lastA = gamepad2.a;

        // Left bumper - Emergency stop
        if (gamepad2.left_bumper && !lastLeftBumper) {
            shootCommand.cancel();
            ilc.forceIdle();
            intake.setIDLE();
        }
        lastLeftBumper = gamepad2.left_bumper;

        // DPAD_UP - Start spinup
        if (gamepad2.dpad_up && !lastDpadUp) {
            ilc.forceIdle();
            ilc.activateILC();
            ilc.startSpinup();
        }
        lastDpadUp = gamepad2.dpad_up;

        // DPAD_DOWN - Manual shoot (if ready)
        if (gamepad2.dpad_down && !lastDpadDown) {
            if (ilc.isReady()) {
                ilc.shoot();
            }
        }
        lastDpadDown = gamepad2.dpad_down;

        // DPAD_RIGHT - Stop shooting
        if (gamepad2.dpad_right && !lastDpadRight) {
            ilc.forceIdle();
        }
        lastDpadRight = gamepad2.dpad_right;

        telemetry.addData("Alliance", alliance);
        telemetry.addLine();

        telemetry.addData("Position", "X: %.1f, Y: %.1f",
                follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();

        telemetry.addData("ILC State", ilc.getState());
        telemetry.addData("ILC Velocity", "%.0f RPM", ilc.getVelocity());
        telemetry.addData("ILC Target", "%.0f RPM", ilc.getTarget());
        telemetry.addData("ILC Ready", ilc.isReady());
        telemetry.addData("ILC Activated", ilc.isILCActivated());
        telemetry.addLine();

        telemetry.addData("Intake State", intake.getIntakeState());
        telemetry.addData("Intake Jammed", intake.intakeJammed);

        telemetry.update();
    }

    @Override
    public void stop() {
        shootCommand.cancel();
        ilc.forceIdle();
        intake.setIDLE();
    }

    // Response Curve Method
    private double applyResponseCurve(double input, double scale) {
        input = Math.max(-1, Math.min(1, input));
        return Math.signum(input) * Math.pow(Math.abs(input), scale);
    }
}