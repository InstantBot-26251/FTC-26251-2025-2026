//package org.firstinspires.ftc.teamcode.chucnotworkingsimple;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.chucnotworkingsimple.Robot;
//import org.firstinspires.ftc.teamcode.util.Alliance;
//
//public class BlueCloseAUto {
//    private Robot r;
//    private MultipleTelemetry multipleTelemetry;
//    private Timer opModeTimer;
//
//    // Starting poses
//    private static final Pose BLUE_STARTING_CLOSE = new Pose(8, 80, Math.toRadians(270));
//    private static final Pose BLUE_STARTING_FAR = new Pose(8, 120, Math.toRadians(270));
//
//    // Scoring position
//    private static final Pose SCORE_POSE = new Pose(16, 128, Math.toRadians(315));
//
//    // Sample intake positions
//    private static final Pose SAMPLE_1 = new Pose(32, 120, Math.toRadians(270));
//    private static final Pose SAMPLE_2 = new Pose(32, 130, Math.toRadians(270));
//    private static final Pose SAMPLE_3 = new Pose(40, 130, Math.toRadians(270));
//
//    // Park position
//    private static final Pose PARK_POSE = new Pose(60, 96, Math.toRadians(180));
//
//    // Auto configuration
//    public final boolean close = true;
//    public final boolean red = false;
//
//    @Override
//    public void runOpMode() {
//        // Initialize
//        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        r = new Robot(hardwareMap, Alliance.BLUE);
//        opModeTimer = new Timer();
//
//        // Set starting pose
//        Pose startPose = BLUE_STARTING_CLOSE;
//        r.follower.setStartingPose(startPose);
//
//        // Build paths
//        PathChain scorePreload = r.follower.pathBuilder()
//                .addPath(r.follower.pathBuilder()
//                        .addBezierLine(startPose, SCORE_POSE)
//                        .setLinearHeadingInterpolation(startPose.getHeading(), SCORE_POSE.getHeading())
//                        .build())
//                .build();
//
//        PathChain toSample1 = r.follower.pathBuilder()
//                .addPath(r.follower.pathBuilder()
//                        .addBezierLine(SCORE_POSE, SAMPLE_1)
//                        .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), SAMPLE_1.getHeading())
//                        .build())
//                .build();
//
//        PathChain scoreSample1 = r.follower.pathBuilder()
//                .addPath(r.follower.pathBuilder()
//                        .addBezierLine(SAMPLE_1, SCORE_POSE)
//                        .setLinearHeadingInterpolation(SAMPLE_1.getHeading(), SCORE_POSE.getHeading())
//                        .build())
//                .build();
//
//        PathChain toSample2 = r.follower.pathBuilder()
//                .addPath(r.follower.pathBuilder()
//                        .addBezierLine(SCORE_POSE, SAMPLE_2)
//                        .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), SAMPLE_2.getHeading())
//                        .build())
//                .build();
//
//        PathChain scoreSample2 = r.follower.pathBuilder()
//                .addPath(r.follower.pathBuilder()
//                        .addBezierLine(SAMPLE_2, SCORE_POSE)
//                        .setLinearHeadingInterpolation(SAMPLE_2.getHeading(), SCORE_POSE.getHeading())
//                        .build())
//                .build();
//
//        PathChain toSample3 = r.follower.pathBuilder()
//                .addPath(r.follower.pathBuilder()
//                        .addBezierLine(SCORE_POSE, SAMPLE_3)
//                        .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), SAMPLE_3.getHeading())
//                        .build())
//                .build();
//
//        PathChain scoreSample3 = r.follower.pathBuilder()
//                .addPath(r.follower.pathBuilder()
//                        .addBezierLine(SAMPLE_3, SCORE_POSE)
//                        .setLinearHeadingInterpolation(SAMPLE_3.getHeading(), SCORE_POSE.getHeading())
//                        .build())
//                .build();
//
//        PathChain park = r.follower.pathBuilder()
//                .addPath(r.follower.pathBuilder()
//                        .addBezierLine(SCORE_POSE, PARK_POSE)
//                        .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), PARK_POSE.getHeading())
//                        .build())
//                .build();
//
//        // Init telemetry
//        multipleTelemetry.addLine("Auto Initialized");
//        multipleTelemetry.addData("Alliance", r.alliance);
//        multipleTelemetry.addData("Starting Pose", startPose);
//        multipleTelemetry.update();
//
//        waitForStart();
//        opModeTimer.resetTimer();
//
//        if (opModeIsActive()) {
//            // === PRELOAD SHOT ===
//            // Drive to score position
//            r.follower.followPath(scorePreload, true);
//            while (opModeIsActive() && r.follower.isBusy()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            // Start spinup while driving
//            r.ilc.startSpinup();
//
//            // Wait for spinup
//            while (opModeIsActive() && !r.ilc.isReady()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            // Shoot preload
//            r.ilc.shoot();
//            sleep(500); // Wait for transfer to complete
//            r.ilc.stopShooting();
//
//            // === SAMPLE 1 CYCLE ===
//            // Drive to sample 1 and intake
//            r.follower.followPath(toSample1, true);
//            r.intake.run();
//            while (opModeIsActive() && r.follower.isBusy()) {
//                r.periodic();
//                updateTelemetry();
//            }
//            sleep(500); // Intake time
//            r.intake.stop();
//
//            // Drive back and shoot
//            r.follower.followPath(scoreSample1, true);
//            r.ilc.startSpinup();
//            while (opModeIsActive() && r.follower.isBusy()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            while (opModeIsActive() && !r.ilc.isReady()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            r.ilc.shoot();
//            sleep(500);
//            r.ilc.stopShooting();
//
//            // === SAMPLE 2 CYCLE ===
//            r.follower.followPath(toSample2, true);
//            r.intake.run();
//            while (opModeIsActive() && r.follower.isBusy()) {
//                r.periodic();
//                updateTelemetry();
//            }
//            sleep(500);
//            r.intake.stop();
//
//            r.follower.followPath(scoreSample2, true);
//            r.ilc.startSpinup();
//            while (opModeIsActive() && r.follower.isBusy()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            while (opModeIsActive() && !r.ilc.isReady()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            r.ilc.shoot();
//            sleep(500);
//            r.ilc.stopShooting();
//
//            // === SAMPLE 3 CYCLE ===
//            r.follower.followPath(toSample3, true);
//            r.intake.run();
//            while (opModeIsActive() && r.follower.isBusy()) {
//                r.periodic();
//                updateTelemetry();
//            }
//            sleep(500);
//            r.intake.stop();
//
//            r.follower.followPath(scoreSample3, true);
//            r.ilc.startSpinup();
//            while (opModeIsActive() && r.follower.isBusy()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            while (opModeIsActive() && !r.ilc.isReady()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            r.ilc.shoot();
//            sleep(500);
//            r.ilc.stopShooting();
//
//            // === PARK ===
//            r.follower.followPath(park, true);
//            while (opModeIsActive() && r.follower.isBusy()) {
//                r.periodic();
//                updateTelemetry();
//            }
//
//            multipleTelemetry.addLine("Auto Complete! 🎉");
//            multipleTelemetry.update();
//        }
//
//        // Save pose for teleop
//        r.stop();
//    }
//
//    private void updateTelemetry() {
//        multipleTelemetry.addData("OpMode Time", "%.2f s", opModeTimer.getElapsedTimeSeconds());
//        multipleTelemetry.addLine();
//
//        multipleTelemetry.addData("ILC State", r.ilc.getState());
//        multipleTelemetry.addData("ILC Ready", r.ilc.isReady());
//        multipleTelemetry.addData("ILC Velocity", "%.0f RPM", r.ilc.getVelocity());
//        multipleTelemetry.addData("ILC Target", "%.0f RPM", r.ilc.getTarget());
//        multipleTelemetry.addLine();
//
//        multipleTelemetry.addData("Follower Busy", r.follower.isBusy());
//        multipleTelemetry.addData("Position", "X: %.1f, Y: %.1f",
//                r.follower.getPose().getX(), r.follower.getPose().getY());
//        multipleTelemetry.addData("Heading", "%.1f°", Math.toDegrees(r.follower.getPose().getHeading()));
//
//        multipleTelemetry.update();
//    }
//}