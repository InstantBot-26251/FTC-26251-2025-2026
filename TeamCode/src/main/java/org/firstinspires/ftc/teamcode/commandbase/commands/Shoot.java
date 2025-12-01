package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;

public class Shoot extends CommandBase {
    private final Shooter shooter;
    private final Intake intake;

    public Shoot() {
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        shooter.setShooter(shooter.getShooterVelocity(), true);
        // Stop Intake to prepare for shooting
        intake.setIntake(IntakeState.TRANSFER);
    }

    @Override
    public void execute() {
        if (shooter.isReadyToShoot()) {
            intake.setIntake(IntakeState.STOP);
        }
        else {
            intake.setIntake(IntakeState.TRANSFER);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntake(IntakeState.STOP);

        shooter.setShooter(shooter.getShooterVelocity(), false);

    }


}
