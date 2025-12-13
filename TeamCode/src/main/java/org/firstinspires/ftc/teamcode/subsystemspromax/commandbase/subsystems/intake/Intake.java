package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Intake extends SubsystemBase {
    private final DcMotorEx intake;
    private final DcMotorEx transfer;

    public boolean intakeJammed = false;

    // NEW: Flag to indicate if ILC has control of transfer
    private boolean transferControlledByILC = false;

    private static IntakeState intakeState;

    private ElapsedTime stateTimer;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        setIDLE();

        stateTimer = new ElapsedTime();
        stateTimer.reset();
    }

    public void periodic() {
        updateIntake();
    }

    // GETTERS
    public IntakeState getIntakeState() {
        return intakeState;
    }

    // NEW: Allow ILC to take control
    public void releaseTransferControl() {
        transferControlledByILC = true;
    }

    // NEW: Intake regains control
    public void resumeTransferControl() {
        transferControlledByILC = false;
    }

    // SETTERS
    public void setIDLE() {
        intake.setPower(IDLE);
        if (!transferControlledByILC) {
            transfer.setPower(IDLE);
        }
        setIntake(IntakeState.IDLE);
    }

    public void setIntake(IntakeState intakeState) {
        Intake.intakeState = intakeState;
        switch (intakeState) {
            case IDLE:
                intake.setPower(IDLE);
                if (!transferControlledByILC) {
                    transfer.setPower(IDLE);
                }
                break;
            case INTAKING:
                intake.setPower(INTAKE);
                if (!transferControlledByILC) {
                    transfer.setPower(INTAKE);
                }
                break;
            case REVERSE:
                intake.setPower(REVERSE);
                if (!transferControlledByILC) {
                    transfer.setPower(REVERSE);
                }
                break;
        }
    }

    public void updateIntake() {
        switch (intakeState) {
            case INTAKING:
                if (intake.isOverCurrent()) {
                    intakeJammed = true;
                    stateTimer.reset();
                    setIntake(IntakeState.REVERSE);
                }
                break;
            case REVERSE:
                if (intakeJammed && stateTimer.milliseconds() >= INTAKE_UNJAM_TIME) {
                    setIntake(IntakeState.INTAKING);
                    intakeJammed = false;
                    stateTimer.reset();
                }
                break;
            case IDLE:
                break;
        }
    }

    public void toggleIntake() {
        if (intakeState.equals(IntakeState.IDLE)) {
            setIntake(IntakeState.INTAKING);
        } else {
            setIntake(IntakeState.IDLE);
        }
    }

    // Allow external control of just the transfer motor
    public void setTransferPower(double power) {
        if (!transferControlledByILC) {
            transfer.setPower(power);
        }
    }

    // Stop intake but allow transfer to continue
    public void stopIntakeOnly() {
        intake.setPower(IDLE);
    }
}