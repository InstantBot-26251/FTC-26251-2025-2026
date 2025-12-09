package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {
    private final DcMotorEx intake;
    private RevColorSensorV3 proximitySensor;

    public boolean intakeJammed = false;
    private int detectionCount = 0;

    private static IntakeState intakeState;

    private ElapsedTime stateTimer;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        proximitySensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");

        setIDLE();

        stateTimer = new ElapsedTime();
        stateTimer.reset();

        intakeState = IntakeState.IDLE;
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
        setIntakeState(IntakeState.IDLE);
    }

    public void setIntake(IntakeState intakeState) {
        Intake.intakeState = intakeState;
        switch (intakeState) {
            case IDLE:
                intake.setPower(IDLE);
                break;
            case INTAKING:
                intake.setPower(INTAKE);
                detectionCount = 0; // Reset count when starting intake
                break;
            case TRANSFERRING:
                intake.setPower(TRANSFER);
                break;
            case REVERSE:
                intake.setPower(REVERSE);
                break;
        }
    }

    public void updateIntake() {
        switch (intakeState) {
            case INTAKING:
                if (transferFull()) {
                    setIntake(IntakeState.IDLE);
                }

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
                // Dont have to set power 0 again
                break;
        }
    }

    public void toggleIntake() {
        if (intakeState.equals(IntakeState.IDLE)) {
            setIntake(IntakeState.INTAKING);
        }
    }

    // STATE MACHINE
    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public boolean transferFull() {
        // Check if sensor detects an object within proximity threshold
        double distance = proximitySensor.getDistance(DistanceUnit.CM);

        if (distance < PROXIMITY_THRESHOLD_CM) {
            detectionCount++;
        }

        // Return true if we've detected 3 or more times
        return detectionCount >= FULL_THRESHOLD;
    }

    // Call this method to reset the detection counter (e.g., after transferring)
    public void resetDetectionCount() {
        detectionCount = 0;
    }

}
