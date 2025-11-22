package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.Queue;

/**
 * Simple rolling average utility class for tracking performance metrics
 */
public class RollingAverage {
    private Queue<Double> values;
    private int maxSize;
    private double sum;

    public RollingAverage(int maxSize) {
        this.maxSize = maxSize;
        this.values = new LinkedList<>();
        this.sum = 0.0;
    }

    public void add(double value) {
        values.add(value);
        sum += value;

        if (values.size() > maxSize) {
            sum -= values.poll();
        }
    }

    public double getAverage() {
        if (values.isEmpty()) {
            return 0.0;
        }
        return sum / values.size();
    }

    public void clear() {
        values.clear();
        sum = 0.0;
    }

    public int getSize() {
        return values.size();
    }

    public boolean isFull() {
        return values.size() >= maxSize;
    }
}