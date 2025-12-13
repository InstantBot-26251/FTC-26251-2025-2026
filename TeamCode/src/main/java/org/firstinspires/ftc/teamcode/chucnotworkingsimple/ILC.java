package org.firstinspires.ftc.teamcode.chucnotworkingsimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class ILC {
    private DcMotorEx ilcLeft, ilcRight;
    private DcMotorEx transfer;

    // Constants
    private static final double TARGET_VELOCITY = 1000; // RPM
    private static final double TRANSFER_REVERSE_POWER = -0.5;
    private static final double TRANSFER_SHOOT_POWER = 1.0;
    private static final double TRANSFER_REVERSE_TIME = 0.3; // seconds
    private static final double FLYWHEEL_MAX_SPINUP_TIME = 2.0; // seconds
    private static final double VELOCITY_TOLERANCE = 50; // RPM

    // PID constants
    private static final double kV = 0.0001;
    private static final double kP = 0.00005;
    private static final double kS = 0.0;

    // State
    public enum ILCState {
        IDLE,
        REVERSING,
        SPINNING_UP,
        READY,
        SHOOTING
    }

    private ILCState state = ILCState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private double targetVelocity = 0;

    public ILC(HardwareMap hardwareMap) {
        ilcLeft = hardwareMap.get(DcMotorEx.class, "ilcL");
        ilcRight = hardwareMap.get(DcMotorEx.class, "ilcR");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        ilcLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setFlywheelPower(0);
        setTransferPower(0);
    }

    // Call this every loop
    public void update() {
        switch (state) {
            case IDLE:
                break;

            case REVERSING:
                if (stateTimer.seconds() >= TRANSFER_REVERSE_TIME) {
                    setTransferPower(0);
                    stateTimer.reset();
                    state = ILCState.SPINNING_UP;
                }
                break;

            case SPINNING_UP:
                updateVelocityPID();
                if (isAtTarget() || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    state = ILCState.READY;
                }
                break;

            case READY:
                updateVelocityPID();
                break;

            case SHOOTING:
                updateVelocityPID();
                break;
        }
    }

    // User commands
    public void startSpinup() {
        if (state == ILCState.IDLE) {
            targetVelocity = TARGET_VELOCITY;
            setTransferPower(TRANSFER_REVERSE_POWER);
            stateTimer.reset();
            state = ILCState.REVERSING;
        }
    }

    public void shoot() {
        if (state == ILCState.READY) {
            setTransferPower(TRANSFER_SHOOT_POWER);
            stateTimer.reset();
            state = ILCState.SHOOTING;
        }
    }

    public void stopShooting() {
        setTransferPower(0);
        setFlywheelPower(0);
        targetVelocity = 0;
        state = ILCState.IDLE;
    }

    public void forceIdle() {
        setTransferPower(0);
        setFlywheelPower(0);
        targetVelocity = 0;
        state = ILCState.IDLE;
    }

    // Getters
    public ILCState getState() {
        return state;
    }

    public boolean isReady() {
        return state == ILCState.READY;
    }

    public boolean isIdle() {
        return state == ILCState.IDLE;
    }

    public double getVelocity() {
        return ilcRight.getVelocity();
    }

    public double getTarget() {
        return targetVelocity;
    }

    private boolean isAtTarget() {
        return Math.abs(targetVelocity - getVelocity()) < VELOCITY_TOLERANCE;
    }

    // Internal methods
    private void setFlywheelPower(double power) {
        ilcLeft.setPower(power);
        ilcRight.setPower(power);
    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    private void updateVelocityPID() {
        double currentVel = getVelocity();
        double error = targetVelocity - currentVel;
        double power = (kV * targetVelocity) + (kP * error) + kS;
        setFlywheelPower(power);
    }
}