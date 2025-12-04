package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.globals.Robot;


public class FollowPathCommand extends CommandBase {
    private final Telemetry telemetry;
    private final Drive drive;
    private final PathChain pathChain;
    private final double maxPower;
    private final boolean holdEnd;

    public FollowPathCommand(Path path) {
        this(path, 1.0);
    }

    public FollowPathCommand(Path path, double maxPower) {
        this(new PathChain(path), false, maxPower);
    }

    public FollowPathCommand(PathChain pathChain) {
        this(pathChain, true, 1.0);
    }

    public FollowPathCommand(PathChain pathChain, boolean holdEnd) {
        this(pathChain, holdEnd, 1.0);
    }

    public FollowPathCommand(PathChain pathChain, boolean holdEnd, double maxPower) {
        telemetry = Robot.getInstance().getTelemetry();
        drive = Drive.getInstance();
        this.pathChain = pathChain;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setMaxPower(maxPower);
        drive.followPath(pathChain, holdEnd);
    }

    @Override
    public void execute() {
        Pose pose = drive.getPoseEstimate();
        telemetry.addLine();
        telemetry.addData("Path Running", drive.isBusy());
        telemetry.addData("Pose", pose.getX() + ", " + pose.getY() + ", " + pose.getHeading());
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}