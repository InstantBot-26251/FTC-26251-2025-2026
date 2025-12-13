package org.firstinspires.ftc.teamcode.chucnotworkingsimple;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Simple TeleTele")
@Config
public class TeleTeleSimple extends OpMode {
    private Robot r;
    private MultipleTelemetry multipleTelemetry;

    // State variables
    public boolean shooting = false;
    public boolean fieldCentric = true;
    public double speed = 1.0;
    public static double targetVelocity = 1000;

    @Override
    public void init() {
        r = new Robot(hardwareMap, Alliance.BLUE);
        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            r.alliance = Alliance.BLUE;
        }

        if (gamepad1.bWasPressed()) {
            r.alliance = Alliance.RED;
        }

        multipleTelemetry.addData("Alliance", r.alliance);
        multipleTelemetry.update();
    }

    @Override
    public void start() {
        r.updateShootTarget();

        if (Robot.endPose == null) {
            r.follower.setStartingPose(r.alliance.equals(Alliance.BLUE) ?
                    r.getDefaultPose().mirror() : r.getDefaultPose());
        } else {
            r.follower.setStartingPose(Robot.endPose);
        }

        r.periodic();
        r.follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        r.periodic();

        // === DRIVE CONTROLS (gamepad1) ===
        double drive = speed * gamepad1.left_stick_y;
        double strafe = speed * gamepad1.left_stick_x;
        double turn = speed * gamepad1.right_stick_x;

        // Apply response curve
        drive = applyResponseCurve(drive, 1.1);
        strafe = applyResponseCurve(strafe, 1.1);
        turn = applyResponseCurve(turn, 2.0);

        if (fieldCentric) {
            r.follower.setTeleOpDrive(drive, strafe, turn, false,
                    r.alliance == Alliance.BLUE ? Math.toRadians(180) : 0);
        } else {
            r.follower.setTeleOpDrive(drive, strafe, turn, true);
        }

        // Slow mode toggle
        if (gamepad1.left_bumper) {
            speed = 0.3;
        } else {
            speed = 1.0;
        }

        // Toggle field centric
        if (gamepad1.dpad_right) {
            fieldCentric = !fieldCentric;
        }

        // Reset pose
        if (gamepad1.dpad_up) {
            if (r.alliance.equals(Alliance.BLUE)) {
                r.follower.setPose(new Pose(32, 30.25, Math.toRadians(180)).mirror());
            } else {
                r.follower.setPose(new Pose(32, 30.25, Math.toRadians(0)));
            }
        }

        // === INTAKE CONTROLS (gamepad1) ===
        if (gamepad1.right_bumper) {
            r.intake.run();
            r.ilc.setTransferPower(1.0);
        } else if (gamepad1.dpad_down) {
            r.intake.reverse();
            r.ilc.setTransferPower(-1.0);
        } else {
            r.intake.stop();
        }

        // === SHOOTING CONTROLS (gamepad1 + gamepad2) ===
        // Toggle shooting mode
        if (gamepad1.b) {
            shooting = !shooting;
        }

        if (shooting) {
            // Start spinup if not already
            if (r.ilc.isIdle()) {
                r.ilc.startSpinup();
            }

            // Shoot when ready and A is pressed
            if (gamepad1.a && r.ilc.isReady()) {
                r.ilc.shoot();
            }
        } else {
            r.ilc.forceIdle();
        }

        // === MANUAL ILC CONTROLS (gamepad2) ===
        // DPAD_UP - Start spinup
        if (gamepad2.dpad_up) {
            r.ilc.startSpinup();
        }

        // DPAD_DOWN - Manual shoot (if ready)
        if (gamepad2.dpad_down) {
            if (r.ilc.isReady()) {
                r.ilc.shoot();
            }
        }

        // DPAD_RIGHT - Stop shooting
        if (gamepad2.dpad_right) {
            r.ilc.stopShooting();
        }

        // A button - Smart shoot
        if (gamepad2.a) {
            if (r.ilc.isIdle()) {
                r.ilc.startSpinup();
            } else if (r.ilc.isReady()) {
                r.ilc.shoot();
            }
        }

        // B button - Toggle intake
        if (gamepad2.b) {
            r.intake.run();
        }

        // X button - Stop intake
        if (gamepad2.x) {
            r.intake.stop();
        }

        // Y button - Reverse
        if (gamepad2.y) {
            r.intake.reverse();
        }

        // Left bumper - Emergency stop
        if (gamepad2.left_bumper) {
            r.ilc.forceIdle();
            r.intake.stop();
            shooting = false;
        }

        // === TELEMETRY ===
        multipleTelemetry.addData("Alliance", r.alliance);
        multipleTelemetry.addLine();

        multipleTelemetry.addData("Position", "X: %.1f, Y: %.1f",
                r.follower.getPose().getX(), r.follower.getPose().getY());
        multipleTelemetry.addData("Heading", "%.1f°",
                Math.toDegrees(r.follower.getPose().getHeading()));
        multipleTelemetry.addLine();

        multipleTelemetry.addData("ILC State", r.ilc.getState());
        multipleTelemetry.addData("ILC Velocity", "%.0f RPM", r.ilc.getVelocity());
        multipleTelemetry.addData("ILC Target", "%.0f RPM", r.ilc.getTarget());
        multipleTelemetry.addData("ILC Ready", r.ilc.isReady());
        multipleTelemetry.addLine();

        multipleTelemetry.addData("Shooting Mode", shooting);
        multipleTelemetry.addData("Field Centric", fieldCentric);
        multipleTelemetry.addData("Speed", speed);

        multipleTelemetry.update();
    }

    @Override
    public void stop() {
        r.stop();
    }

    // Response Curve Method
    private double applyResponseCurve(double input, double scale) {
        input = Math.max(-1, Math.min(1, input));
        return Math.signum(input) * Math.pow(Math.abs(input), scale);
    }
}