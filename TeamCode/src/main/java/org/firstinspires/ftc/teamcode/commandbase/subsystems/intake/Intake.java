package org.firstinspires.ftc.teamcode.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.IntakeConstants.*;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.globals.Enigma;
import org.firstinspires.ftc.teamcode.globals.RobotMap;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;

public class Intake extends SubsystemTemplate {
    private DcMotorEx intake;
    private RevColorSensorV3 proximitySensor;

    private Telemetry telemetry;

    public ElapsedTime intakeTimer;

    public boolean intakeJammed = false;
    public static IntakeState intakeState = IntakeState.STOP;

    private int detectionCount = 0;

    private static final Intake INSTANCE = new Intake();

    public static Intake getInstance() {
        return INSTANCE;
    }

    public Intake() {
        intakeTimer = new ElapsedTime();
        intakeTimer.reset();
    }

    @Override
    public void onAutonomousInit() {
//        telemetry = Enigma.getInstance().getTelemetry();
        initHardware();
    }

    @Override
    public void onTeleopInit() {
//        telemetry = Enigma.getInstance().getTelemetry();
        initHardware();
    }

    @Override
    public void periodic() {
        updateIntake();
    }

    public void initHardware() {
        RobotMap map = RobotMap.getInstance();

        intake = map.INTAKE;
        proximitySensor = map.PROXIMITY_SENSOR;

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setIntake(IntakeState intakeState) {
        switch (intakeState) {
            case STOP:
                intake.setPower(0);
                break;
            case TRANSFER:
                intake.setPower(INTAKE_TRANSFER_SPEED);
                break;
            case FORWARD:
                intake.setPower(INTAKE_FORWARD_SPEED);
                detectionCount = 0; // Reset count when starting intake
                break;
            case REVERSE:
                intake.setPower(INTAKE_REVERSE_SPEED);
                break;
        }
    }

    private void toggleIntake() {
        if (intakeState.equals(IntakeState.STOP)) {
            setIntake(IntakeState.FORWARD);
        }
    }

    private void updateIntake() {
        switch (intakeState) {
            case FORWARD:
                if (transferFull()) {
                    setIntake(IntakeState.STOP);
                }

                if ((intake.isOverCurrent())) {
                    intakeJammed = true;
                    intakeTimer.reset();
                    setIntake(IntakeState.REVERSE);
                }
                break;

            case REVERSE:
                if (intakeJammed && intakeTimer.milliseconds() >= INTAKE_UNJAM_TIME) {
                    setIntake(IntakeState.FORWARD);
                    intakeJammed = false;
                    intakeTimer.reset();
                }
                break;
            case STOP:
                // No point of setting intakeMotor to 0 again
                break;
        }
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

    public static SequentialCommandGroup ActiveStopIntake() {
        return new SequentialCommandGroup(
                new SetIntake(IntakeState.FORWARD),
                new WaitCommand(500),
                new SetIntake(IntakeState.STOP)
        );
    }
}

