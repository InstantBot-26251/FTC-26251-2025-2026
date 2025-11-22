package org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter;

import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A lookup table that uses monotone cubic spline interpolation to compute
 * smooth values between discrete control points.
 * Courtesy of Claude Sonnet 4.1
 */
public class InterpolatingLUT {

    private List<Double> mX = new ArrayList<>();
    private List<Double> mY = new ArrayList<>();
    private List<Double> mM = new ArrayList<>();


    private final List<Double> inputValues;
    private final List<Double> outputValues;
    private final List<Double> tangents;

    /**
     * Constructs an empty interpolating lookup table.
     * Use add() to insert control points, then call build() to prepare for queries.
     */
    public InterpolatingLUT() {
        this.inputValues = new ArrayList<>();
        this.outputValues = new ArrayList<>();
        this.tangents = new ArrayList<>();
    }

    /**
     * Constructs a lookup table from parallel lists of inputs and outputs.
     * The table is immediately ready for use.
     */
    public InterpolatingLUT(List<Double> inputs, List<Double> outputs) {
        validateControlPoints(inputs, outputs);

        this.inputValues = new ArrayList<>(inputs);
        this.outputValues = new ArrayList<>(outputs);
        this.tangents = new ArrayList<>();

        computeSplineCoefficients();
    }

    /**
     * Adds a data point to the lookup table.
     * Call build() after adding all points to prepare the spline.
     */
    public InterpolatingLUT add(double input, double output) {
        inputValues.add(input);
        outputValues.add(output);
        return this;
    }

    /**
     * Finalizes the lookup table by computing spline interpolation coefficients.
     * This must be called after adding control points and before querying values.
     */
    public InterpolatingLUT build() {
        validateControlPoints(inputValues, outputValues);
        computeSplineCoefficients();
        return this;
    }

    /**
     * Creates a monotone cubic spline from a given set of control points.
     *
     * <p>
     * The spline is guaranteed to pass through each control point exactly. Moreover, assuming the control points are
     * monotonic (Y is non-decreasing or non-increasing) then the interpolated values will also be monotonic.
     *
     * @throws IllegalArgumentException if the X or Y arrays are null, have different lengths or have fewer than 2 values.
     * @throws IllegalArgumentException if the X values are not strictly increasing.
     * @return this class (for chaining calls)
     */
    //public static LUTWithInterpolator createLUT(List<Double> x, List<Double> y) {
    public InterpolatingLUT createLUT() {
        List<Double> x = this.mX;
        List<Double> y = this.mY;

        if (x == null || y == null || x.size() != y.size() || x.size() < 2) {
            throw new IllegalArgumentException("There must be at least two control "
                    + "points and the arrays must be of equal length.");
        }

        final int n = x.size();
        Double[] d = new Double[n - 1]; // could optimize this out
        Double[] m = new Double[n];

        // Compute slopes of secant lines between successive points.
        for (int i = 0; i < n - 1; i++) {
            Double h = x.get(i + 1) - x.get(i);
            if (h <= 0f) {
                throw new IllegalArgumentException("The control points must all "
                        + "have strictly increasing X values.");
            }
            d[i] = (y.get(i + 1) - y.get(i)) / h;
        }

        // Initialize the tangents as the average of the secants.
        m[0] = d[0];
        for (int i = 1; i < n - 1; i++) {
            m[i] = (d[i - 1] + d[i]) * 0.5f;
        }
        m[n - 1] = d[n - 2];

        // Update the tangents to preserve monotonicity.
        for (int i = 0; i < n - 1; i++) {
            if (d[i] == 0f) { // successive Y values are equal
                m[i] = Double.valueOf(0f);
                m[i + 1] = Double.valueOf(0f);
            } else {
                double a = m[i] / d[i];
                double b = m[i + 1] / d[i];
                double h = Math.hypot(a, b);
                if (h > 9f) {
                    double t = 3f / h;
                    m[i] = t * a * d[i];
                    m[i + 1] = t * b * d[i];
                }
            }
        }
        mX = x;
        mY = y;
        mM = Arrays.asList(m);

        return this;
    }

    /**
     * Retrieves the interpolated output value for a given input.
     * Throws an exception if the input is outside the defined range.
     */
    public double get(double input) {
        if (Double.isNaN(input)) {
            return input;
        }

        int size = inputValues.size();
        double minInput = inputValues.get(0);
        double maxInput = inputValues.get(size - 1);

        if (input < minInput || input > maxInput) {
            throw new IllegalArgumentException(
                    String.format("Input %.3f is outside valid range [%.3f, %.3f]",
                            input, minInput, maxInput));
        }

        // Find the segment containing this input value
        int segmentIndex = locateSegment(input);

        // Check for exact match
        if (input == inputValues.get(segmentIndex)) {
            return outputValues.get(segmentIndex);
        }

        // Perform Hermite interpolation
        return evaluateHermiteSpline(segmentIndex, input);
    }

    private void validateControlPoints(List<Double> inputs, List<Double> outputs) {
        if (inputs == null || outputs == null) {
            throw new IllegalArgumentException("Control point lists cannot be null");
        }
        if (inputs.size() != outputs.size()) {
            throw new IllegalArgumentException("Input and output lists must have equal length");
        }
        if (inputs.size() < 2) {
            throw new IllegalArgumentException("At least two control points are required");
        }
    }

    private void computeSplineCoefficients() {
        int numPoints = inputValues.size();
        Double[] secantSlopes = new Double[numPoints - 1];
        Double[] computedTangents = new Double[numPoints];

        // Calculate slopes between consecutive points
        for (int i = 0; i < numPoints - 1; i++) {
            double deltaX = inputValues.get(i + 1) - inputValues.get(i);
            if (deltaX <= 0.0) {
                throw new IllegalArgumentException(
                        "Input values must be in strictly increasing order");
            }
            double deltaY = outputValues.get(i + 1) - outputValues.get(i);
            secantSlopes[i] = deltaY / deltaX;
        }

        // Initialize tangents at each point
        computedTangents[0] = secantSlopes[0];
        for (int i = 1; i < numPoints - 1; i++) {
            computedTangents[i] = (secantSlopes[i - 1] + secantSlopes[i]) * 0.5;
        }
        computedTangents[numPoints - 1] = secantSlopes[numPoints - 2];

        // Adjust tangents to maintain monotonicity
        for (int i = 0; i < numPoints - 1; i++) {
            if (Math.abs(secantSlopes[i]) < 1e-10) {
                // Flat segment - zero out tangents
                computedTangents[i] = 0.0;
                computedTangents[i + 1] = 0.0;
            } else {
                double alpha = computedTangents[i] / secantSlopes[i];
                double beta = computedTangents[i + 1] / secantSlopes[i];
                double magnitude = Math.sqrt(alpha * alpha + beta * beta);

                if (magnitude > 9.0) {
                    double scaleFactor = 3.0 / magnitude;
                    computedTangents[i] = scaleFactor * alpha * secantSlopes[i];
                    computedTangents[i + 1] = scaleFactor * beta * secantSlopes[i];
                }
            }
        }

        tangents.clear();
        for (Double tangent : computedTangents) {
            tangents.add(tangent);
        }
    }

    private int locateSegment(double input) {
        int index = 0;
        while (index < inputValues.size() - 1 && input >= inputValues.get(index + 1)) {
            index++;
        }
        return index;
    }

    private double evaluateHermiteSpline(int index, double input) {
        double x0 = inputValues.get(index);
        double x1 = inputValues.get(index + 1);
        double y0 = outputValues.get(index);
        double y1 = outputValues.get(index + 1);
        double m0 = tangents.get(index);
        double m1 = tangents.get(index + 1);

        double intervalWidth = x1 - x0;
        double normalizedPos = (input - x0) / intervalWidth;
        double t2 = normalizedPos * normalizedPos;
        double t3 = t2 * normalizedPos;

        // Hermite basis functions
        double h00 = 2 * t3 - 3 * t2 + 1;
        double h10 = t3 - 2 * t2 + normalizedPos;
        double h01 = -2 * t3 + 3 * t2;
        double h11 = t3 - t2;

        return h00 * y0 + h10 * intervalWidth * m0 + h01 * y1 + h11 * intervalWidth * m1;
    }

    @Override
    public String toString() {
        StringBuilder result = new StringBuilder("InterpolatingLUT[");
        for (int i = 0; i < inputValues.size(); i++) {
            if (i > 0) result.append(", ");
            result.append(String.format("(%.2f → %.2f, m=%.2f)",
                    inputValues.get(i), outputValues.get(i), tangents.get(i)));
        }
        result.append("]");
        return result.toString();
    }
}