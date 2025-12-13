package org.firstinspires.ftc.teamcode.subsystemspromax.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands.ShootSequenceCommand;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc.InertialLaunchCore;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.Intake;

@Autonomous(name = "Auto with ILC", group = "Auto", preselectTeleOp = "TeleOp")
public class AutoILC extends OpMode {
    private Follower follower;
    private InertialLaunchCore ilc;
    private Intake intake;

    private Timer opModeTimer;

    // Poses
    private final Pose startPose = new Pose(20.5, 122.5, Math.toRadians(90.0));
    private final Pose scorePose = new Pose(48, 96, Math.toRadians(135));
    private final Pose intake1Pose = new Pose(20, 84, Math.toRadians(180));
    private final Pose intake1CP = new Pose(60.31813361611877, 81.23860021208908);
    private final Pose intake2Pose = new Pose(20, 60, Math.toRadians(180));
    private final Pose intake2CP = new Pose(75.74125132555673, 55.43160127253447);
    private final Pose intake3Pose = new Pose(20, 36, Math.toRadians(180));
    private final Pose intake3CP = new Pose(92.997, 30.235);

    private PathChain scorePreload, intake1, score1, intake2, score2, intake3, score3;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake1CP, intake1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, intake1CP, scorePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake2CP, intake2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, intake2CP, scorePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), scorePose.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake3CP, intake3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(intake3Pose, intake3CP, scorePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), scorePose.getHeading())
                .build();
    }


     // Creates a command sequence for driving to score position and shooting and params are path and timeout and it will return the command group that drives and shoots
    public SequentialCommandGroup pathShoot(PathChain path, long timeout) {
        return new SequentialCommandGroup(
                // Drive to score position while starting spinup
                new ParallelCommandGroup(
                        new DriveTo(follower, path).withTimeout(timeout),
                        new InstantCommand(() -> ilc.startSpinup())
                                .andThen(new WaitCommand(100)) // Small delay to start spinup
                ),

                // Wait for ILC to be ready
                new WaitUntilCommand(() -> ilc.isReady()).withTimeout(2000),

                // Execute the shot
                new ShootSequenceCommand(ilc, intake)
        );
    }


    // Creates a command sequence for driving to intake position and intaking the params are path, timeout and it will return the command group that drives and intakes
    public SequentialCommandGroup pathIntake(PathChain path, long timeout) {
        return new SequentialCommandGroup(
                // Drive to intake position while intaking
                new ParallelCommandGroup(
                        new DriveTo(follower, path).withTimeout(timeout),
                        new IntakeCommand(intake)
                )
        );
    }

    @Override
    public void init() {
        // Setup telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        opModeTimer = new Timer();

        // Initialize subsystems
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        ilc = new InertialLaunchCore(hardwareMap, intake);


        // Build paths
        buildPaths();
        follower.setStartingPose(startPose);

        // Register subsystems with command scheduler
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(intake);

        telemetry.addLine("Auto Initialized");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();
    }

    @Override
    public void init_loop() {
//        telemetry.addData("ILC Distance", ilc.getDistance());
//        telemetry.addData("ILC ID", ilc.getID());
        telemetry.addData("Position", follower.getPose());
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();

        // Schedule the complete autonomous sequence
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // Burner command (sometimes needed for initialization)
                        new InstantCommand(),

                        // Preload shot
                        pathShoot(scorePreload, 1500),

                        // First cycle: intake → score
                        pathIntake(intake1, 1867),
                        pathShoot(score1, 2250),

                        // Second cycle: intake → score
                        pathIntake(intake2, 2267),
                        pathShoot(score2, 3000),

                        // Third cycle: intake → score
                        pathIntake(intake3, 2267),
                        pathShoot(score3, 3000),

                        // Final message
                        new InstantCommand(() -> telemetry.addLine("Auto Complete! 🎉"))
                )
        );
    }

    @Override
    public void loop() {
        // Update all subsystems
        follower.update();
        ilc.periodic();
        intake.periodic();

        // Run command scheduler
        CommandScheduler.getInstance().run();

        // Telemetry
        telemetry.addData("OpMode Time", "%.2f s", opModeTimer.getElapsedTimeSeconds());
        telemetry.addLine();

        telemetry.addData("ILC State", ilc.getState());
        telemetry.addData("ILC Ready", ilc.isReady());
        telemetry.addData("ILC Target RPM", "%.0f", ilc.getTarget());
        telemetry.addData("ILC Actual RPM", "%.0f", ilc.getVelocity());
//        telemetry.addData("Distance to Target", "%.1f in", ilc.getDistance());
        telemetry.addLine();

        telemetry.addData("Intake State", intake.getIntakeState());
        telemetry.addData("Intake Jammed", intake.intakeJammed);
        telemetry.addLine();

        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Position", "X: %.1f, Y: %.1f",
                follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up
        CommandScheduler.getInstance().reset();
        ilc.forceIdle();
        intake.setIDLE();
    }
}