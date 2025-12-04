package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Do this to tune
 * First I shall tune feedforward  first : kS and kV
 * Then, I shall increase kP until oscillations appear, then ill reduce by 50%
 * Them if steady state errors or smth like that keeps happening, add small kI
 */
public class FlywheelSpECialPID {

    private double m_kP;
    private double m_kI;
    private double m_kD;
    private double m_kF; // This is kV -velocity feedforward

    private double m_kS; // Static friction voltage
    private double m_kA; // Acceleration gain (OPTIONAL)

    private double m_setpoint;
    private double m_measurement;

    private double m_prevError;
    private double m_totalError;

    // Tolerance for determining if at setpoint
    private double m_positionTolerance = Double.POSITIVE_INFINITY;
    private double m_velocityTolerance = Double.POSITIVE_INFINITY;

    // Integral windup prevention
    private double m_iZone = Double.POSITIVE_INFINITY;
    private double m_maxIntegral = Double.POSITIVE_INFINITY;

    // For output clamping
    private double m_minOutput = -1.0;
    private double m_maxOutput = 1.0;

    // Timing
    private double m_period; // I think we should have this

    /**
     * Constructor
     * @param kP Proportional gain - try starting around 0.0001-0.001
     * @param kI Integral gain - 0 but if needed do smth like 0.0001
     * @param kD Derivative gain - 0
     * @param kF Velocity feedforward - start around 0.00008-0.0001
     * @param period The period (in seconds) of the control loop
     */
    public FlywheelSpECialPID(double kP, double kI, double kD, double kF, double period) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
        m_kF = kF;
        m_kS = 0.0;
        m_kA = 0.0;
        m_period = period;

        reset();
    }
    /**
     * Constructor but with feedforward and the extra gains
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kS Static friction gain
     * @param kV Velocity gain
     * @param kA Acceleration gain
     */
    public FlywheelSpECialPID(double kP, double kI, double kD, double kS, double kV, double kA, double period) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
        m_kF = kV; // kV and kF are the same thing
        m_kS = kS;
        m_kA = kA;
        m_period = period;

        reset();
    }


    /**
     * Constructor from PIDFCoefficients object (FTC SDK standard)
     * @param coefficients PIDFCoefficients from FTC SDK
     * @param period The period (in seconds) of the control loop
     */
    public FlywheelSpECialPID(PIDFCoefficients coefficients, double period) {
        this(coefficients.p, coefficients.i, coefficients.d, coefficients.f, period);
    }

    /**
     * Constructor from PIDFCoefficients with default period of 0.02s (50Hz)
     * @param coefficients PIDFCoefficients from FTC SDK
     */
    public FlywheelSpECialPID(PIDFCoefficients coefficients) {
        this(coefficients, 0.02);
    }

    /**
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feedforward gain
     */
    public void setPIDF(double kP, double kI, double kD, double kF) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
        m_kF = kF;
    }


    /**
     * @param kS Static friction voltage
     * @param kV Velocity gain (same as kF)
     * @param kA Acceleration gain
     */
    public void setFF(double kS, double kV, double kA) {
        m_kS = kS;
        m_kF = kV; // kV and kF are the same
        m_kA = kA;
    }

    public void setFF(double kS, double kV) {
        setFF(kS, kV, 0.0);
    }

    // Individual setters for P,I,D,F,S,V, or A
    public void setP(double kP) { m_kP = kP; }
    public void setI(double kI) { m_kI = kI; }
    public void setD(double kD) { m_kD = kD; }
    public void setF(double kF) { m_kF = kF; }
    public void setS(double kS) { m_kS = kS; }
    public void setV(double kV) { m_kF = kV; } // kV and kF are the same
    public void setA(double kA) { m_kA = kA; }

    // Individual getters for P,I,D,F,S,V, or A
    public double getP() { return m_kP; }
    public double getI() { return m_kI; }
    public double getD() { return m_kD; }
    public double getF() { return m_kF; }
    public double getS() { return m_kS; }
    public double getV() { return m_kF; } // kV and kF are the same
    public double getA() { return m_kA; }



     // Sets the setpoint for the controller - haha i love this name
    public void setSetPoint(double setpoint) {
        m_setpoint = setpoint;
    }




    public double getSetPoint() {
        return m_setpoint;
    }


    public boolean atSetPoint() {
        double positionError = m_setpoint - m_measurement;
        double velocityError = (positionError - m_prevError) / m_period;

        return Math.abs(positionError) < m_positionTolerance
                && Math.abs(velocityError) < m_velocityTolerance;
    }

    /**
     * @param positionTolerance Position error tolerance (e.g., 50 ticks/sec)
     * @param velocityTolerance Velocity error tolerance
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_positionTolerance = positionTolerance; // For above method atsetpoint
        m_velocityTolerance = velocityTolerance;
    }

    /**
     * Sets the position tolerance for determining if at setpoint.
     * Velocity tolerance is set to infinity.
     * @param positionTolerance Position error tolerance
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }


    public double getPositionError() {
        return m_setpoint - m_measurement;
    }

    public double getVelocityError() {
        double positionError = m_setpoint - m_measurement;
        return (positionError - m_prevError) / m_period;
    }

    /**
     * Calculates the control output.
     * @param measurement The current measurement of the process variable
     * @param setpoint The desired setpoint
     * @return The calculated control output (feedforward + PID feedback)
     */
    public double calculate(double measurement, double setpoint) {
        m_setpoint = setpoint;
        return calculate(measurement);
    }

    /**
     * Calculates the control output using the previously set setpoint.
     * @param measurement The current measurement of the process variable
     * @return The calculated control output (feedforward + PID feedback)
     */
    public double calculate(double measurement) {
        m_measurement = measurement;
        double error = m_setpoint - measurement;

        // Calculate feedforward component
        double feedforward = calculateFeedforward(m_setpoint);

        // I-Zone: reset integral if error is too large
        if (Math.abs(error) > m_iZone) {
            m_totalError = 0;
        } else {
            // Accumulate error for integral term
            m_totalError += error * m_period;

            // Clamp integral to prevent windup
            m_totalError = Math.max(-m_maxIntegral, Math.min(m_maxIntegral, m_totalError));
        }

        // Calculate derivative (rate of change of error)
        double derivative = (error - m_prevError) / m_period;

        // Store error for next iteration
        m_prevError = error;

        // Calculate PID feedback output
        double feedback = m_kP * error + m_kI * m_totalError + m_kD * derivative;

        // Combine feedforward and feedback
        double output = feedforward + feedback;

        // Clamp output to specified range
        return Math.max(m_minOutput, Math.min(m_maxOutput, output));
    }

    /**
     * @param setpoint The desired setpoint
     * @return The feedforward voltage
     */
    public double calculateFeedforward(double setpoint) {
        // Calculate acceleration (change in setpoint)
        double acceleration = (setpoint - m_setpoint) / m_period;

        // Feedforward equation: kS * sgn(velocity) + kF * velocity + kA * acceleration
        double ks_term = m_kS * Math.signum(setpoint);
        double kf_term = m_kF * setpoint;
        double ka_term = m_kA * acceleration;

        return ks_term + kf_term + ka_term;
    }

    /**
     * Resets the controller state.
     * Clears the integral accumulator and previous error.
     * Call this when changing setpoints or re-enabling the controller.
     */
    public void reset() {
        m_prevError = 0;
        m_totalError = 0;
        m_measurement = 0;
    }
}