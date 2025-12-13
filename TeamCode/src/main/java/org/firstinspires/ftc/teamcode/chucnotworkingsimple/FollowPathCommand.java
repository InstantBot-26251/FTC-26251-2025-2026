package org.firstinspires.ftc.teamcode.chucnotworkingsimple;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;
    private double maxPower = 1;
    private boolean robotCentric = false;

    public FollowPathCommand(Robot robot, PathChain pathChain) {
        this.follower = robot.follower;
        this.path = pathChain;
    }

    public FollowPathCommand(Robot robot, PathChain pathChain, double maxPower) {
        this.follower = robot.follower;
        this.path = pathChain;
        this.maxPower = maxPower;
    }

    public FollowPathCommand(Robot robot, PathChain pathChain, boolean holdEnd) {
        this.follower = robot.follower;
        this.path = pathChain;
        this.holdEnd = holdEnd;
    }

    public FollowPathCommand(Robot robot, PathChain pathChain, boolean holdEnd, double maxPower) {
        this.follower = robot.follower;
        this.path = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    // Decides if robot should hold end
    public FollowPathCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    // Sets max power for path
    public FollowPathCommand setMaxPower(double power) {
        this.maxPower = power;
        return this;
    }

    @Override
    public void initialize() {
        follower.setMaxPower(maxPower);
        follower.followPath(path);
    }

    @Override
    public void execute() {
        follower.followPath(path, maxPower, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
