package org.firstinspires.ftc.teamcode.subsystemspromax.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc.InertialLaunchCore;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeState;

@TeleOp(name = "Auto with ILC", group = "Auto")
public class AutoILC extends OpMode {
    private Follower follower;
    private InertialLaunchCore ilc;
    private Intake intake;

    private Timer pathTimer, opModeTimer, shootTimer;

    public enum PathState {
        // Preload sequence
        DRIVE_TO_SHOOT_PRELOAD,
        SPINUP_PRELOAD,
        SHOOT_PRELOAD,

        // First intake/score cycle
        INTAKING_1,
        DRIVE_TO_SCORE_1,
        SPINUP_1,
        SHOOT_1,

        // Second intake/score cycle
        INTAKING_2,
        DRIVE_TO_SCORE_2,
        SPINUP_2,
        SHOOT_2,

        // Third intake/score cycle
        INTAKING_3,
        DRIVE_TO_SCORE_3,
        SPINUP_3,
        SHOOT_3,

        DONE
    }

    private PathState pathState;

    private final Pose startPose = new Pose(20.5, 122.5, Math.toRadians(90.0));
    private final Pose scorePose = new Pose(48, 96, Math.toRadians(135));
    private final Pose intake1Pose = new Pose(20, 84, Math.toRadians(180));
    private final Pose intake1CP = new Pose(60.31813361611877, 81.23860021208908);
    private final Pose intake2Pose = new Pose(20, 60, Math.toRadians(180));
    private final Pose intake2CP = new Pose(75.74125132555673, 55.43160127253447);
    private final Pose intake3Pose = new Pose(20, 36, Math.toRadians(180));
    private final Pose intake3CP = new Pose(92.997, 30.235);

    private PathChain scorePreload, intake1, score1, intake2, score2, intake3, score3;

    private static final double TRANSFER_RUN_TIME = 1.0; // Time to run transfer in seconds

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        intake1CP,
                        intake1Pose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intake1Pose,
                        intake1CP,
                        scorePose
                ))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        intake2CP,
                        intake2Pose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intake2Pose,
                        intake2CP,
                        scorePose
                ))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), scorePose.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        intake3CP,
                        intake3Pose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intake3Pose,
                        intake3CP,
                        scorePose
                ))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT_PRELOAD:
                follower.followPath(scorePreload, true);
                ilc.startSpinup(); // Start spinning up while driving
                setPathState(PathState.SPINUP_PRELOAD);
                break;

            case SPINUP_PRELOAD:
                // Update velocity as we get closer
                ilc.updateVelocityDuringSpinup();

                // Wait for both: at position AND flywheels ready
                if (!follower.isBusy() && ilc.isReady()) {
                    ilc.shoot(); // Open gate
                    intake.setIntake(IntakeState.TRANSFERRING); // Start transfer
                    shootTimer.resetTimer();
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                // Wait for transfer to complete
                if (shootTimer.getElapsedTimeSeconds() > TRANSFER_RUN_TIME) {
                    ilc.stopShooting(); // Close gate, stop flywheels
                    intake.setIDLE(); // Stop transfer
                    intake.resetDetectionCount(); // Reset for next intake

                    follower.followPath(intake1, true);
                    intake.setIntake(IntakeState.INTAKING);
                    setPathState(PathState.INTAKING_1);
                }
                break;

            // FIRST CYCLE
            case INTAKING_1:
                // When path finishes, we've intaken all artifacts hopefully
                if (!follower.isBusy()) {
                    intake.setIDLE(); // Stop intake
                    follower.followPath(score1, true);
                    ilc.startSpinup(); // Start spinning while driving
                    setPathState(PathState.DRIVE_TO_SCORE_1);
                }
                break;

            case DRIVE_TO_SCORE_1:
                ilc.updateVelocityDuringSpinup();

                if (!follower.isBusy()) {
                    setPathState(PathState.SPINUP_1);
                }
                break;

            case SPINUP_1:
                // If not ready yet, wait (usually already ready from drive)
                if (ilc.isReady()) {
                    ilc.shoot();
                    intake.setIntake(IntakeState.TRANSFERRING);
                    shootTimer.resetTimer();
                    setPathState(PathState.SHOOT_1);
                }
                break;

            case SHOOT_1:
                if (shootTimer.getElapsedTimeSeconds() > TRANSFER_RUN_TIME) {
                    ilc.stopShooting();
                    intake.setIDLE();
                    intake.resetDetectionCount();

                    // Start driving to intake AND start intaking immediately
                    follower.followPath(intake2, true);
                    intake.setIntake(IntakeState.INTAKING);
                    setPathState(PathState.INTAKING_2);
                }
                break;

            // SECOND CYCLE
            case INTAKING_2:
                // When path finishes, we've intaken all balls
                if (!follower.isBusy()) {
                    intake.setIDLE();
                    follower.followPath(score2, true);
                    ilc.startSpinup();
                    setPathState(PathState.DRIVE_TO_SCORE_2);
                }
                break;

            case DRIVE_TO_SCORE_2:
                ilc.updateVelocityDuringSpinup();

                if (!follower.isBusy()) {
                    setPathState(PathState.SPINUP_2);
                }
                break;

            case SPINUP_2:
                if (ilc.isReady()) {
                    ilc.shoot();
                    intake.setIntake(IntakeState.TRANSFERRING);
                    shootTimer.resetTimer();
                    setPathState(PathState.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (shootTimer.getElapsedTimeSeconds() > TRANSFER_RUN_TIME) {
                    ilc.stopShooting();
                    intake.setIDLE();
                    intake.resetDetectionCount();

                    // Start driving to intake AND start intaking immediately
                    follower.followPath(intake3, true);
                    intake.setIntake(IntakeState.INTAKING);
                    setPathState(PathState.INTAKING_3);
                }
                break;

            // THIRD CYCLE
            case INTAKING_3:
                // When path finishes, we've intaken all balls
                if (!follower.isBusy()) {
                    intake.setIDLE();
                    follower.followPath(score3, true);
                    ilc.startSpinup();
                    setPathState(PathState.DRIVE_TO_SCORE_3);
                }
                break;

            case DRIVE_TO_SCORE_3:
                ilc.updateVelocityDuringSpinup();

                if (!follower.isBusy()) {
                    setPathState(PathState.SPINUP_3);
                }
                break;

            case SPINUP_3:
                if (ilc.isReady()) {
                    ilc.shoot();
                    intake.setIntake(IntakeState.TRANSFERRING);
                    shootTimer.resetTimer();
                    setPathState(PathState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (shootTimer.getElapsedTimeSeconds() > TRANSFER_RUN_TIME) {
                    ilc.stopShooting();
                    intake.setIDLE();
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("Auto Finished YAYAYYAYA");
                break;

            default:
                telemetry.addLine("No Path State Commanded");
                break;
        }
    }

    public void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        ilc = new InertialLaunchCore(hardwareMap);
        intake = new Intake(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

        setPathState(PathState.DRIVE_TO_SHOOT_PRELOAD);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        // Update both subsystems every loop
        follower.update();
        ilc.periodic();
        intake.periodic();

        statePathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("ILC State", ilc.getState());
        telemetry.addData("Intake State", intake.getIntakeState());
        telemetry.addData("ILC Ready", ilc.isReady());
        telemetry.addData("ILC Target RPM", ilc.getTarget());
        telemetry.addData("ILC Actual RPM", ilc.getVelocity());
        telemetry.addData("Distance to Target", ilc.getDistance());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("OpMode Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}