package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Lebron {
    public Follower follower;
    private int index;


    public static Pose startPose =  new Pose(30.250, 133.200, Math.toRadians(90)).mirror();
    public  Pose scorePose =  new Pose(18.000, 84.000, Math.toRadians(135)).mirror();
    public  Pose scoreControlPoint = new Pose(55.593, 94.779).mirror();
    public  Pose intake1Pose = new Pose(48.000, 96.000, Math.toRadians(180)).mirror();
    public  Pose intake1ControlPoint =   new Pose(74.367, 82.613).mirror();
    public  Pose intake2Pose = new Pose(18.000, 60.000, Math.toRadians(180)).mirror();
    public  Pose intake2ControlPoint = new Pose(79.712, 55.432).mirror();
    public  Pose intake3Pose = new Pose(17.256, 35.275, Math.toRadians(180)).mirror();
    public  Pose intake3ControlPoint = new Pose(92.997, 30.235).mirror();

    public Lebron(HardwareMap hardwareMap) {
        Constants.createFollower(hardwareMap);

            startPose = startPose;
            scoreControlPoint = scoreControlPoint;
            scorePose = scorePose;
            intake1Pose = intake1Pose;
            intake1ControlPoint = intake1ControlPoint;
            intake2Pose = intake2Pose;
            intake2ControlPoint = intake2ControlPoint;
            intake3Pose = intake3Pose;
            intake3ControlPoint = intake3ControlPoint;

            index = 0;
    }

    public PathChain scorePreload() {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                scoreControlPoint,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public PathChain intake1() { // intake 1
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                intake1ControlPoint,
                                intake1Pose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                               intake1Pose,
                               scoreControlPoint,
                               scorePose
                        )
                )
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public PathChain intake2() {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                intake2ControlPoint,
                                intake2Pose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading(), 0.5)
                .build();
    }

    public PathChain score2() {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intake2Pose,
                                scoreControlPoint,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), scorePose.getHeading())
                .build();
    }


//    public PathChain scoreGate() {
//        return f.pathBuilder()
//                .addPath(new BezierCurve(gate, gateControl, score))
//                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
//                .build();
//    }

    public PathChain intake3() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake3ControlPoint, intake3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading(), 0.7)
                .build();
    }

    public PathChain score3() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(intake3Pose, scoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), scorePose.getHeading())
                .build();
    }


    public PathChain next() {
        switch (index++) {
            case 0: return scorePreload();
            case 1: return intake1();
            case 2: return score1();
            case 3: return intake2();
            case 5: return score2();
            case 6: return intake3();
            case 7: return score3();
            default: return null;
        }
    }

    public boolean hasNext() {
        int PATH_COUNT = 9;
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}
