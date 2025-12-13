package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

public class DriveTo extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean hasStarted = false;

    public DriveTo(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
    }

    @Override
    public void initialize() {
        follower.followPath(path, true);
        hasStarted = true;
    }

    @Override
    public void execute() {
        // Follower updates happen in main loop
    }

    @Override
    public boolean isFinished() {
        return hasStarted && !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
        }
    }
}