//package org.firstinspires.ftc.teamcode.util;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.RunCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.util.TelemetryData;
//
//
//import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import static org.firstinspires.ftc.teamcode.util.Alliance.*;
//
//public abstract class AutoTemplate extends CommandOpMode {
//    protected final Robot robot = Robot.getInstance();
//    protected Follower follower;
//    protected TelemetryData telemetryData;
//
//    boolean lastUp;
//    boolean lastDown;
//    boolean lastX;
//    boolean lastY;
//    boolean lastB;
//
//    protected Alliance alliance;
//
//    @Override
//    public void initialize() {
//        super.reset();
//
//        telemetryData = new TelemetryData(telemetry);
//        robot.autonomousInit(telemetry, hardwareMap);
//
//        // Initialize follower
//        follower = Constants.createFollower(hardwareMap);
//
//        // Configure auto variables
//        while (!isStarted() && !isStopRequested()) {
//            config();
//
//            if (gamepad1.options) {
//                break;
//            }
//        }
//
//        follower.setStartingPose(getStartingPose());
//        buildPaths();
//
//        // Set default alliance if none selected
//        if (alliance == NONE) {
//            alliance = RED;
//        }
//
//        // Display ready status
//        telemetryData.addData("Status", "Initialized, Ready to start");
//        telemetryData.update();
//
//
//        // Schedule the autonomous sequence with delay
//        schedule(
//                new RunCommand(() -> follower.update()),
//                new WaitCommand((long) 0.8),
//                makeAutoSequence()
//        );
//    }
//
//    @Override
//    public void run() {
//        super.run();
//
//        // Update telemetry with robot pose
//        telemetryData.addData("X", follower.getPose().getX());
//        telemetryData.addData("Y", follower.getPose().getY());
//        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
//        telemetryData.update();
//    }
//
//    public void config() {
//
//        // Select alliance
//        if (checkInputs(gamepad1.x, lastX)) {
//            switch(alliance) {
//                case NONE:
//                case RED:
//                    alliance = BLUE;
//                    break;
//                case BLUE:
//                    alliance = RED;
//                    break;
//            }
//        }
//
//
//        // Set old inputs
//        lastUp = gamepad1.dpad_up;
//        lastDown = gamepad1.dpad_down;
//        lastX = gamepad1.x;
//        lastB = gamepad1.b;
//        lastY = gamepad1.y;
//
//        telemetry.addData("Status", "Configuring Autonomous");
//        telemetry.addData("Controls", "\nSelect alliance: X \nConfirm: OPTIONS");
//        telemetry.addLine();
//        telemetry.update();
//    }
//
//    public boolean checkInputs(boolean current, boolean last) {
//        return (last != current) && current;
//    }
//
//    protected abstract Pose getStartingPose();
//
//    protected abstract void buildPaths();
//
//    protected abstract SequentialCommandGroup makeAutoSequence();
//
//}