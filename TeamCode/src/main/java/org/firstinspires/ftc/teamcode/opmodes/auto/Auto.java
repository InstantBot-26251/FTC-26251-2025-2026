package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.Enigma;
import org.firstinspires.ftc.teamcode.paths.Lebron;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Lebron Auto", group = "Auto")
public class Auto extends OpMode {
    private Follower follower;
    private ElapsedTime pathTimer;

    private enum PathState {
        SCOREP,
        INTAKE1,
        SCORE1,
        INTAKE2,
        SCORE2,
        INTAKE3,
        SCORE3,
        FINISHED
    }

    private PathState state;

    public static Pose startPose =  new Pose(30.250, 133.200, Math.toRadians(90)).mirror();
    public  Pose scorePose =  new Pose(18.000, 84.000, Math.toRadians(135)).mirror();
    public  Pose scoreControlPoint = new Pose(55.593, 94.779).mirror();
    public  Pose intake1Pose = new Pose(48.000, 96.000, Math.toRadians(180)).mirror();
    public  Pose intake1ControlPoint =   new Pose(74.367, 82.613).mirror();
    public  Pose intake2Pose = new Pose(18.000, 60.000, Math.toRadians(180)).mirror();
    public  Pose intake2ControlPoint = new Pose(79.712, 55.432).mirror();
    public  Pose intake3Pose = new Pose(17.256, 35.275, Math.toRadians(180)).mirror();
    public  Pose intake3ControlPoint = new Pose(92.997, 30.235).mirror();

    public PathChain drivetoShoot;
    public PathChain intake1;
    public PathChain scoreIntake1;
    public PathChain intake2;
    public PathChain scoreIntake2;
    public PathChain intake3;
    public PathChain scoreIntake3;

    public void buildPaths() {
        drivetoShoot = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, scoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 1)
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake1ControlPoint, intake1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading(),0.3)
                .build();

        scoreIntake1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intake1Pose,
                                scoreControlPoint,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading(),1)
                .build();

        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                intake2ControlPoint,
                                intake2Pose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading(), 0.5)
                .build();

        scoreIntake2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                intake2ControlPoint,
                                intake2Pose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading(), 1)
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake3ControlPoint, intake3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading(), 0.7)
                .build();

        scoreIntake3 = follower.pathBuilder() .addPath(new BezierCurve(intake3Pose, scoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), scorePose.getHeading(), 1)
                .build();
    }

    public void statePathUpdate() {
        switch (state) {
            case SCOREP:
                follower.followPath(drivetoShoot, true);
                if (!follower.isBusy()) { state = PathState.INTAKE1; }
                break;
//
//            case INTAKE1:
//                follower.followPath(intake1, true);
//
//                state = PathState.SCORE1;
//                break;
//
//            case SCORE1:
//                follower.followPath(scoreIntake1, true);
//                state = PathState.INTAKE2;
//                break;
//
//            case INTAKE2:
//                follower.followPath(intake2, true);
//                state = PathState.SCORE2;
//                break;
//
//            case SCORE2:
//                follower.followPath(scoreIntake2,true);
//                state = PathState.INTAKE3;
//                break;
//
//            case INTAKE3:
//                follower.followPath(intake3, true);
//                state = PathState.SCORE3;
//                break;
//
//            case SCORE3:
//                follower.followPath(scoreIntake3, true);
//                state = PathState.FINISHED;
//                break;
//        }
    }}

    public void setPathState(PathState newState) {
        state = newState;
    }

    @Override
    public void init() {
        state = PathState.SCOREP;
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void start() {
        setPathState(PathState.SCOREP);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // Telemetry
        telemetry.addData("State", state);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}