//package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//
//import org.firstinspires.ftc.teamcode.subsystemspromax.RobotE;
//
//public class FollowPathCommand extends Command {
//    private final Follower follower;
//    private final PathChain path;
//    private boolean holdEnd = true;
//    private double maxPower = 1;
//    private boolean robotCentric = false;
//
//    public FollowPathCommand(RobotE robot, PathChain pathChain) {
//        this.follower = robot.follower;
//        this.path = pathChain;
//    }
//
//    public FollowPathCommand(RobotE robot, PathChain pathChain, double maxPower) {
//        this.follower = robot.follower;
//        this.path = pathChain;
//        this.maxPower = maxPower;
//    }
//
//    public FollowPathCommand(RobotE robot, PathChain pathChain, boolean holdEnd) {
//        this.follower = robot.follower;
//        this.path = pathChain;
//        this.holdEnd = holdEnd;
//    }
//
//    public FollowPathCommand(RobotE robot, PathChain pathChain, boolean holdEnd, double maxPower) {
//        this.follower = robot.follower;
//        this.path = pathChain;
//        this.holdEnd = holdEnd;
//        this.maxPower = maxPower;
//    }
//
//    // Decides if robot should hold end
//    public FollowPathCommand setHoldEnd(boolean holdEnd) {
//        this.holdEnd = holdEnd;
//        return this;
//    }
//
//    // Sets max power for path
//    public FollowPathCommand setMaxPower(double power) {
//        this.maxPower = power;
//        return this;
//    }
//
//    @Override
//    public void start() {
//        follower.followPath(path, maxPower, holdEnd);
//    }
//
//    @Override
//    public boolean done() {
//        return !follower.isBusy();
//    }
//}
