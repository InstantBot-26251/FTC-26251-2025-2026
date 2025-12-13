//package org.firstinspires.ftc.teamcode.solverslib.commandbase.commands;
//
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.seattlesolvers.solverslib.command.CommandBase;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;
//
//
//public class FollowPathCommand extends CommandBase {
//    private final Telemetry telemetry;
////    private final Drive drive;
//    private final Robot robot;
//    private final PathChain pathChain;
//    private final double maxPower;
//    private final boolean holdEnd;
//
//    public FollowPathCommand(Path path) {
//        this(path, 1.0);
//    }
//
//    public FollowPathCommand(Path path, double maxPower) {
//        this(new PathChain(path), false, maxPower);
//    }
//
//    public FollowPathCommand(PathChain pathChain) {
//        this(pathChain, true, 1.0);
//    }
//
//    public FollowPathCommand(PathChain pathChain, boolean holdEnd) {
//        this(pathChain, holdEnd, 1.0);
//    }
//
//    public FollowPathCommand(PathChain pathChain, boolean holdEnd, double maxPower) {
//        telemetry = Robot.getInstance().getTelemetry();
////        drive = Drive.getInstance();
//        robot = Robot.getInstance();
//        this.pathChain = pathChain;
//        this.maxPower = maxPower;
//        this.holdEnd = holdEnd;
//
//        addRequirements(robot.drive);
//    }
//
//    @Override
//    public void initialize() {
//        robot.drive.setMaxPower(maxPower);
//        robot.drive.followPath(pathChain, holdEnd);
//    }
//
//    @Override
//    public void execute() {
//        Pose pose = robot.drive.getPoseEstimate();
//        telemetry.addLine();
//        telemetry.addData("Path Running", robot.drive.isBusy());
//        telemetry.addData("Pose", pose.getX() + ", " + pose.getY() + ", " + pose.getHeading());
//    }
//
//    @Override
//    public boolean isFinished() {
//        return !robot.drive.isBusy();
//    }
//}