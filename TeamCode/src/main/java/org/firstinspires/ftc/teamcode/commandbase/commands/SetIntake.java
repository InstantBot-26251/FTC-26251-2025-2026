package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.MotorState;
import org.firstinspires.ftc.teamcode.globals.Enigma;

public class SetIntake extends CommandBase {
    private final Enigma robot;
    private final MotorState motorState;
    private boolean waitForArtifacts;

    private ElapsedTime timer;

    public SetIntake(MotorState motorState) {
        this(motorState, false);
    }

    public SetIntake(MotorState motorState, boolean waitForArtifacts) {
        robot = Enigma.getInstance();
        this.motorState = motorState;
        this.waitForArtifacts = waitForArtifacts;

        timer = new ElapsedTime();

        addRequirements(robot.intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        robot.intake.setIntake(motorState);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (waitForArtifacts && robot.intake.transferFull()) {
            return true;
        }

        return !waitForArtifacts && timer.milliseconds() > 200;
    }
}