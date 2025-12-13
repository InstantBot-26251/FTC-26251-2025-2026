//package org.firstinspires.ftc.teamcode.solverslib.opmodes.test.shooter;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;
//
//@Config
//@TeleOp(name = "Shooter Tuning", group = "Test")
//public class ShooterPIDTuner extends OpMode {
//
//    public static int TARGET_VELOCITY = 1500;
//
//    private Robot robot;
//    private boolean shooterRunning = false;
//
//    private boolean prevA = false;
//    private boolean prevB = false;
//
//    @Override
//    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        robot = Robot.getInstance();
//
//        robot.teleopInit(telemetry, hardwareMap);
//    }
//
//    @Override
//    public void loop() {
//        // Start shooter at target velocity
//        boolean currentA = gamepad1.a;
//        if (currentA && !prevA) {
//            shooterRunning = true;
//            robot.ilc.setArbalestVelocity(TARGET_VELOCITY);
//        }
//        prevA = currentA;
//
//        // Stop shooter
//        boolean currentB = gamepad1.b;
//        if (currentB && !prevB) {
//            shooterRunning = false;
//            robot.ilc.stopILC();
//        }
//        prevB = currentB;
//
//        // Update target velocity if shooter is running
//        if (shooterRunning) {
//            robot.ilc.setArbalestVelocity(TARGET_VELOCITY);
//        }
//
//        // Display telemetry
//        double currentVel = robot.ilc.getShooterVelocity();
//        double error = TARGET_VELOCITY - currentVel;
//        double percentError = TARGET_VELOCITY > 0 ? (error / TARGET_VELOCITY) * 100 : 0;
//
//        telemetry.addLine("=== SHOOTER STATUS ===");
//        telemetry.addData("Running", shooterRunning ? "YES" : "NO");
//        telemetry.addData("Target Velocity", "%.0f ticks/sec", TARGET_VELOCITY);
//        telemetry.addData("Current Velocity", "%.0f ticks/sec", currentVel);
//        telemetry.addData("Error", "%.0f ticks/sec (%.1f%%)", error, percentError);
//        telemetry.addData("Ready", robot.ilc.isReadyToShoot() ? "YES" : "NO");
//        telemetry.update();
//    }
//
//}