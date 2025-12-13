package org.firstinspires.ftc.teamcode.subsystemspromax.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class Auto extends OpMode {
    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_TO_SHOOT_P,
        SHOOT_PRELOAD,
        PARK
    }

    private PathState pathState;

    private final Pose startPose = new Pose(20.5, 122.5, Math.toRadians(90));
    private final Pose scorePose = new Pose(96, 96, Math.toRadians(55));
    private final Pose park = new Pose(96, 96, Math.toRadians(55));

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
