package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.subsystemspromax.RobotE;
import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeState;

public class Intake extends Command {
    private final RobotE robot;

    public Intake(RobotE robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.intake.setIntake(IntakeState.IDLE); // Get intake ready to intake
    }

    @Override
    public void execute() {
        robot.intake.setIntake(IntakeState.INTAKING); // Intake artifacts
    }

    @Override
    public boolean done() {
        return robot.intake.transferFull();
    }
}
