//package org.firstinspires.ftc.teamcode.solverslib.commandbase.commands;
//
//import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.drive.DriveConstants.GOAL_POSE_X;
//import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.drive.DriveConstants.GOAL_POSE_Y;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.RobotLog;
//import com.seattlesolvers.solverslib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;
//
//public class AimRobot extends CommandBase {
////    private final Drive drive;
////    private final ATVision vision;
//
//    private final Robot robot;
//
//    private final ElapsedTime timer;
//    private final ElapsedTime stageTimer;
//
//    private final double goalX;
//    private final double goalY;
//
//    private double kP = 3.0;
//    private int stage = 0;
//
//    // Stage constants
//    private static final int STAGE_AIM_DRIVETRAIN = 0;
//    private static final int STAGE_COMPLETE = 1;
//
//    // Tolerances and timeouts
//    private static final double HEADING_TOLERANCE = Math.toRadians(2.0);
//
//    private double targetDistance = 0.0;
//    private double targetHeading = 0.0;
//    private boolean usingOdometry = false;
//
//    public AimRobot() {
////        drive = Drive.getInstance();
////        vision = ATVision.getInstance();
//        robot = Robot.getInstance();
//
//
//        this.timer = new ElapsedTime();
//        this.stageTimer = new ElapsedTime();
//
//        goalX = GOAL_POSE_X;
//        goalY = GOAL_POSE_Y;
//
//        addRequirements(robot.drive, robot.ilc);
//    }
//
//    @Override
//    public void initialize() {
//        stage = STAGE_AIM_DRIVETRAIN;
//        timer.reset();
//        stageTimer.reset();
//        usingOdometry = false;
//
//        RobotLog.aa("AimCommand", "Initialized staged auto-aim sequence");
//    }
//
//    @Override
//    public void execute() {
//        Pose pose = robot.drive.getPose();
//        double robotX = pose.getX();
//        double robotY = pose.getY();
//        double robotHeading = pose.getHeading();
//
//        // Calculate target heading based on current position (always updated)
//        targetHeading = Math.atan2(goalY - robotY, goalX - robotX);
//        double headingError = angleWrap(targetHeading - robotHeading);
//
//        // Calculate distance using vision first, fallback to O T O S
//        targetDistance = getTargetDistance(pose);
//
//        switch (stage) {
//            case STAGE_AIM_DRIVETRAIN:
//                executeAimDrivetrain(headingError);
//                break;
//
//            case STAGE_COMPLETE:
//                // Do nothing, waiting for prev command to finish
//                break;
//        }
//    }
//
//    // Get target distance - prioritize vision, fallback to odometry
//    private double getTargetDistance(Pose pose) {
//        // Try to get vision distance first
//        double visionDistance = robot.ilc.getTargetDistance();
//
//        if (visionDistance > 0) {
//            // Valid vision reading
//            usingOdometry = false;
//            return visionDistance;
//        } else {
//            // Vision failed, use odometry
//            usingOdometry = true;
//            double dx = goalX - pose.getX();
//            double dy = goalY - pose.getY();
//            return Math.sqrt(dx * dx + dy * dy);
//        }
//    }
//
//    // Apply heading lock - calculates turn power to maintain aim at goal
//    private double calculateHeadingLock(double headingError) {
//        double turn = kP * headingError;
//        // Clamp turn speed so robot doesn't explode
//        return Math.max(-0.5, Math.min(0.5, turn));
//    }
//
//    private void executeAimDrivetrain(double headingError) {
//        double turn = calculateHeadingLock(headingError);
//
//        robot.drive.driveFieldCentric(0.0, 0.0, turn);
//
//        RobotLog.aa("AimCommand", String.format("Stage 0: Aiming - error=%.3f rad, turn=%.3f, dist=%.2f, using_odom=%b",
//                headingError, turn, targetDistance, usingOdometry));
//
//        // Check if aligned
//        if (Math.abs(headingError) < HEADING_TOLERANCE) {
//            advanceStage();
//        }
//    }
//
//    private void advanceStage() {
//        stage++;
//        stageTimer.reset();
//        RobotLog.aa("AimCommand", "Advanced to stage " + stage);
//    }
//
//    private static double angleWrap(double angle) {
//        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
//        while (angle < -Math.PI) angle += 2.0 * Math.PI;
//        return angle;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        robot.drive.driveFieldCentric(0.0, 0.0, 0.0);
//    }
//}
