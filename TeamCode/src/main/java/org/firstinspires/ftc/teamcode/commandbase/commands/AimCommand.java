package org.firstinspires.ftc.teamcode.commandbase.commands;


import static org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveConstants.*;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.globals.Enigma;

public class AimCommand extends CommandBase {

    private final Enigma robot;
    private final Drive drive;


    private final double goalX;
    private final double goalY;


    private double kP = 3.0;

    private static final double HEADING_TOLERANCE = Math.toRadians(2.0);

    public AimCommand() {
        robot = Enigma.getInstance();
        drive = Drive.getInstance();

        goalX = GOAL_POSE_X;
        goalY = GOAL_POSE_Y;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        RobotLog.aa("DriveAim", "Initialized drive-only auto aim");
    }

    @Override
    public void execute() {
        Pose pose = drive.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        double targetHeading = Math.atan2(goalY - robotY, goalX - robotX);

        double error = angleWrap(targetHeading - robotHeading);

        double turn = kP * error;


        if (turn > 0.5) turn = 0.5;
        if (turn < -0.5) turn = -0.5;

        drive.driveFieldCentric(0.0, 0.0, turn);

        RobotLog.aa("DriveAim", String.format("err=%.3f rad, turn=%.3f", error, turn));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop rotating when done
        drive.driveFieldCentric(0.0, 0.0, 0.0);
        RobotLog.aa("DriveAim", "Ended (interrupted=" + interrupted + ")");
    }

    @Override
    public boolean isFinished() {
        Pose pose = drive.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        double targetHeading = Math.atan2(goalY - robotY, goalX - robotX);
        double error = angleWrap(targetHeading - robotHeading);

        return Math.abs(error) < HEADING_TOLERANCE;
    }


    public void setKP(double kP) {
        this.kP = kP;
    }

    private static double angleWrap(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
