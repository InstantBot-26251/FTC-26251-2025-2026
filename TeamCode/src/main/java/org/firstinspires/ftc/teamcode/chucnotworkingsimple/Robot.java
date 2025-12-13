package org.firstinspires.ftc.teamcode.chucnotworkingsimple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

public class Robot {
    // Subsystems
    public final Intake intake;
    public final ILC ilc;
    public final Follower follower;

    // Hardware
    private final List<LynxModule> hubs;

    // Alliance
    public Alliance alliance;

    // State
    public static Pose endPose;
    public static Pose defaultPose = new Pose(32, 30.25, 0);
    public static Pose shootTarget = new Pose(6, 138, 0);

    private final Timer loopTimer = new Timer();

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        // Initialize subsystems
        intake = new Intake(hardwareMap);
        ilc = new ILC(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // Initialize hardware
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        loopTimer.resetTimer();
        updateShootTarget();
    }

    public void periodic() {
        // Clear bulk cache every loop
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // Update subsystems
        follower.update();
        ilc.update();

        // Update shoot target based on alliance
        updateShootTarget();
    }

    public void stop() {
        endPose = follower.getPose();
        ilc.forceIdle();
        intake.stop();
    }

    public void updateShootTarget() {
        if (alliance == Alliance.BLUE && shootTarget.getX() != 6) {
            shootTarget = new Pose(6, 138, 0);
        } else if (alliance == Alliance.RED && shootTarget.getX() != 138) {
            shootTarget = new Pose(138, 138, 0);
        }
    }

    public Pose getShootTarget() {
        return shootTarget;
    }

    public Pose getDefaultPose() {
        return defaultPose;
    }
}