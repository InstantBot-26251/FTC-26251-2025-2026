package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Intake extends SubsystemBase {
    private final DcMotorEx intake;
    private final DcMotorEx transfer;

    public boolean intakeJammed = false;

    private static IntakeState intakeState;

    private ElapsedTime stateTimer;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        setIDLE();

        stateTimer = new ElapsedTime();
        stateTimer.reset();

    }

    public void periodic() {
        updateIntake();
    }

    // GETTERS
    // STATE MACHINE
    public IntakeState getIntakeState() {
        return intakeState;
    }


    // SETTERS
    public void setIDLE() {
        intake.setPower(IDLE);
        setIntake(IntakeState.IDLE);
    }

    public void setIntake(IntakeState intakeState) {
        Intake.intakeState = intakeState;
        switch (intakeState) {
            case IDLE:
                intake.setPower(IDLE);
                transfer.setPower(IDLE);
                break;
            case INTAKING:
                intake.setPower(INTAKE);
                transfer.setPower(INTAKE);
                break;
            case REVERSE:
                intake.setPower(REVERSE);
                transfer.setPower(REVERSE);
                break;
        }
    }

    public void updateIntake() {
        switch (intakeState) {
            case INTAKING:
                if ((intake.isOverCurrent())) {
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
                // Don't have to set power 0 again
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
    // This is useful when ILC needs to take over transfer control
    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    // Stop intake but allow transfer to continue (for shooting)
    public void stopIntakeOnly() {
        intake.setPower(IDLE);
    }


}
