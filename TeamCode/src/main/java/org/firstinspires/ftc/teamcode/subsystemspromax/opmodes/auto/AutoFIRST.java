package org.firstinspires.ftc.teamcode.subsystemspromax.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto FIRST", group = "Auto")
public class AutoFIRST extends OpMode {
    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_TO_SHOOT_P,
        SHOOT_PRELOAD,
        INTAKE_FIRST,
        SCORE_FIRST_INTAKE_SET,
        INTAKE_SECOND,
        SCORE_SECOND_INTAKE_SET,
        INTAKE_THIRD,
        SCORE_THIRD_INTAKE_SET
    }

    private PathState pathState;

    private final Pose startPose = new Pose(120, 122.5, Math.toRadians(90));
    private final Pose scorePose = new Pose(96, 96, Math.toRadians(55));
    private final Pose intake1Pose = new Pose(126.50467289719626, 83.66355140186916, Math.toRadians(0));
    private final Pose intake1CP = new Pose(81.64485981308411, 80.97196261682242);
    private final Pose intake2Pose = new Pose(127.4018691588785, 59.663551401869164, Math.toRadians(0));
    private final Pose intake2CP = new Pose(75.74125132555673, 55.43160127253447);
    private final Pose intake3Pose = new Pose(20, 36, Math.toRadians(180));
    private final Pose intake3CP = new Pose(58.76635514018692, 28.03738317757009);

    private PathChain scorePreload;
    private PathChain intake1;
    private PathChain score1;
    private PathChain intake2;
    private PathChain score2;
    private PathChain intake3;
    private PathChain score3;

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
            case DRIVE_TO_SHOOT_P:
                follower.followPath(scorePreload, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO: Add in flywheel logic
                    follower.followPath(intake1, true);
                    setPathState(PathState.INTAKE_FIRST);
                    telemetry.addLine("Done Path 1");
                }
                break;

            case INTAKE_FIRST:
                if (!follower.isBusy()) {
                    follower.followPath(score1, true);
                    telemetry.addLine("Intaked first set or done with path 2");
                    setPathState(PathState.SCORE_FIRST_INTAKE_SET);
                }
                break;

            case SCORE_FIRST_INTAKE_SET:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO: Add in flywheel logic
                    follower.followPath(intake2, true);
                    setPathState(PathState.INTAKE_SECOND);
                    telemetry.addLine("Scored first set or done with path 3");
                }
                break;

            case INTAKE_SECOND:
                if (!follower.isBusy()) {
                    follower.followPath(score2, true);
                    setPathState(PathState.SCORE_SECOND_INTAKE_SET);
                    telemetry.addLine("Intaked second set or done with path 4");
                }
                break;

            case SCORE_SECOND_INTAKE_SET:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO: Add in flywheel logic
                    follower.followPath(intake3, true);
                    setPathState(PathState.INTAKE_THIRD);
                    telemetry.addLine("Scored second set or done with path 5");
                }
                break;

            case INTAKE_THIRD:
                if (!follower.isBusy()) {
                    follower.followPath(score3, true);
                    setPathState(PathState.SCORE_THIRD_INTAKE_SET);
                    telemetry.addLine("Intaked third set or done with path 6");
                }
                break;

            case SCORE_THIRD_INTAKE_SET:
                if (!follower.isBusy()) {
                    // TODO: Add in flywheel logic
                    telemetry.addLine("Scored third set or done with path 7");
                    telemetry.addLine("Auto Finished YAY you better have score all 9 artifacts now get ready to cook");
                }
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

        setPathState(PathState.DRIVE_TO_SHOOT_P);

        follower = Constants.createFollower(hardwareMap);
        // TODO: Add in other subsystems

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();


        telemetry.addData("Path State: ", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Time: ", opModeTimer.getElapsedTimeSeconds());
    }
}
