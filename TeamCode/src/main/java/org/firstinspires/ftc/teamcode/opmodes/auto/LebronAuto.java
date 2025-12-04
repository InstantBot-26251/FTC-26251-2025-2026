package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.Shoot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.util.AutoTemplate;

import static org.firstinspires.ftc.teamcode.util.Alliance.*;

@Autonomous(name = "Lebron Auto", group = "Auto")
public class LebronAuto extends AutoTemplate {

    // Poses
    private Pose startPose;
    private Pose scorePose;
    private Pose scoreControlPoint;
    private Pose intake1Pose;
    private Pose intake1ControlPoint;
    private Pose intake2Pose;
    private Pose intake2ControlPoint;
    private Pose intake3Pose;
    private Pose intake3ControlPoint;

    // Path chains
    private PathChain drivetoShoot;
    private PathChain intake1;
    private PathChain scoreIntake1;
    private PathChain intake2;
    private PathChain scoreIntake2;
    private PathChain intake3;
    private PathChain scoreIntake3;

    @Override
    protected Pose getStartingPose() {
        // Initialize poses based on alliance
        startPose = new Pose(30.250, 133.200, Math.toRadians(90));
        scorePose = new Pose(18.000, 84.000, Math.toRadians(135));
        scoreControlPoint = new Pose(55.593, 94.779);
        intake1Pose = new Pose(48.000, 96.000, Math.toRadians(180));
        intake1ControlPoint = new Pose(74.367, 82.613);
        intake2Pose = new Pose(18.000, 60.000, Math.toRadians(180));
        intake2ControlPoint = new Pose(79.712, 55.432);
        intake3Pose = new Pose(17.256, 35.275, Math.toRadians(180));
        intake3ControlPoint = new Pose(92.997, 30.235);

        // Mirror poses if BLUE alliance
        if (alliance == RED) {
            startPose = startPose.mirror();
            scorePose = scorePose.mirror();
            scoreControlPoint = scoreControlPoint.mirror();
            intake1Pose = intake1Pose.mirror();
            intake1ControlPoint = intake1ControlPoint.mirror();
            intake2Pose = intake2Pose.mirror();
            intake2ControlPoint = intake2ControlPoint.mirror();
            intake3Pose = intake3Pose.mirror();
            intake3ControlPoint = intake3ControlPoint.mirror();
        }

        return startPose;
    }

    @Override
    protected void buildPaths() {
        drivetoShoot = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, scoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 1)
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake1ControlPoint, intake1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading(), 0.3)
                .build();

        scoreIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, scoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading(), 1)
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake2ControlPoint, intake2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading(), 0.5)
                .build();

        scoreIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, scoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), scorePose.getHeading(), 1)
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake3ControlPoint, intake3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading(), 0.7)
                .build();

        scoreIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(intake3Pose, scoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), scorePose.getHeading(), 1)
                .build();
    }

    @Override
    protected SequentialCommandGroup makeAutoSequence() {
        return new SequentialCommandGroup(
                // Score preload
                new FollowPathCommand(drivetoShoot, true),
                scoreArtifacts(),
                new WaitCommand(500),

                // First intake cycle
                new FollowPathCommand(intake1, true),
                intakeArtifacts(),
                new FollowPathCommand(scoreIntake1, true),
                scoreArtifacts(),
                new WaitCommand(300),

                // Second intake cycle
                new FollowPathCommand(intake2, true),
                intakeArtifacts(),
                new FollowPathCommand(scoreIntake2, true),
                scoreArtifacts(),
                new WaitCommand(300),

                // Third intake cycle
                new FollowPathCommand(intake3, true),
                intakeArtifacts(),
                new FollowPathCommand(scoreIntake3, true),
                scoreArtifacts()
        );
    }



    // Mechanism commands - replace with your actual subsystem commands
    private SequentialCommandGroup scoreArtifacts() {
        return new SequentialCommandGroup(
                // Add scoring commands here
                 new InstantCommand(() -> new Shoot())
        );
    }

    private SequentialCommandGroup intakeArtifacts() {
        return new SequentialCommandGroup(
                // Add intake commands here
                new InstantCommand(() -> Intake.getInstance().setIntake(IntakeState.FORWARD))
        );
    }
}